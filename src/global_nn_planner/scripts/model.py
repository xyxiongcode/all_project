import torch
import torch.nn as nn
import torch.nn.functional as F


class BasicResidualBlock(nn.Module):
    """Basic residual block with two 3x3 conv layers (channels constant) and skip connection."""

    def __init__(self, channels: int):
        super(BasicResidualBlock, self).__init__()
        self.conv1 = nn.Conv2d(channels, channels, kernel_size=3, stride=1, padding=1)
        self.conv2 = nn.Conv2d(channels, channels, kernel_size=3, stride=1, padding=1)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x):
        # First conv + ReLU
        out = self.relu(self.conv1(x))
        # Second conv
        out = self.conv2(out)
        # Add skip connection (input x) and apply ReLU
        out = self.relu(out + x)
        return out


class VIN(nn.Module):
    """
    Value Iteration Network (VIN) implementation in PyTorch.
    This network performs differentiable planning via value iteration.
    Expects input as an image (e.g. obstacle map with goal) and uses an initial reward conv + iterative conv for planning.
    If start coordinates are provided, it will select the Q-values at that position for output.
    """

    def __init__(self, input_channels: int = 2, state_dim: int = 64, num_actions: int = 8, k: int = 20):
        """
        :param input_channels: Number of input channels (e.g. obstacle + goal maps).
        :param state_dim: Spatial width/height of input maps (assumed square).
        :param num_actions: Number of possible actions (directions) to output.
        :param k: Number of value iteration iterations.
        """
        super(VIN, self).__init__()
        self.num_actions = num_actions
        self.k = k
        # Convolution to compute initial reward map (1 channel)
        self.conv_r = nn.Conv2d(input_channels, 1, kernel_size=3, stride=1, padding=1)
        # Convolution for transition (should output num_actions channels). We set bias=False since it's used iteratively.
        self.conv_q = nn.Conv2d(1, num_actions, kernel_size=3, stride=1, padding=1, bias=False)
        # Note: We do not include a softmax here; will use raw Q values for loss.

    def forward(self, x, x2=None, coords=None):
        """
        Forward pass for VIN.
        :param x: Input tensor of shape (N, C, H, W) representing the environment (and goal).
        :param x2: (Unused for VIN, kept for interface compatibility)
        :param coords: Tensor of shape (N, 2) giving start positions (row, col) for each sample (optional, used to gather Q at those coords).
        :return: Tensor of shape (N, num_actions) with Q-values for each action at the start state.
        """
        # Compute reward map from input
        r = self.conv_r(x)  # shape: (N, 1, H, W)
        # Initialize value map V as zero
        v = torch.zeros_like(r)  # shape: (N, 1, H, W)
        # Perform value iteration updates
        for _ in range(self.k):
            # Compute Q-values for all actions: convolve value map with transition kernel and add reward
            q = self.conv_q(v) + r  # shape: (N, num_actions, H, W)
            # Take max over actions (dim=1) to update value map
            v, _ = torch.max(q, dim=1, keepdim=True)  # shape: (N, 1, H, W)
        # After K iterations, compute final Q (for output policy)
        q = self.conv_q(v) + r  # shape: (N, num_actions, H, W)
        if coords is not None:
            # Gather Q-values at the provided start coordinates for each sample
            # coords is assumed to be in the same spatial scale as input x
            coords = coords.long()
            N = q.size(0)
            # Ensure coords are within bounds of Q map
            # If input was downsampled within VIN, scale coordinates accordingly (not needed here as VIN keeps full resolution).
            coords_clamped = torch.clamp(coords, 0, q.shape[2] - 1)
            # Advanced indexing to select [batch_index, action, y_coord, x_coord] for each batch
            q_at_start = q[torch.arange(N, device=q.device), :, coords_clamped[:, 0],
                         coords_clamped[:, 1]]  # shape: (N, num_actions)
            return q_at_start  # logits for each action at start state
        else:
            # If no coords given, return the full Q-map (N, num_actions, H, W) or the max Q per action map
            # Here we return the max-pooled Q over spatial dimensions as a fallback
            q_max = torch.amax(q, dim=(2, 3))  # shape: (N, num_actions)
            return q_max


class DBCNN(nn.Module):
    """
    Dual-Branch Convolutional Neural Network (DB-CNN) implementation.
    This model has two branches:
      - Branch One: global feature extraction with conv, residual blocks, pooling, and fully connected layers.
      - Branch Two: local feature extraction with conv, residual blocks (no downsampling beyond initial preprocessing), producing a spatial value function map.
    The outputs of the two branches are combined to predict the next action.
    """

    def __init__(self, input_channels: int = 2, num_actions: int = 8):
        """
        :param input_channels: Number of input channels for the imagery (e.g. obstacle map, target map, etc.).
        :param num_actions: Number of output action classes.
        """
        super(DBCNN, self).__init__()
        # ----- Preprocessing layers (shared by both branches) -----
        self.conv00 = nn.Conv2d(input_channels, 6, kernel_size=5, stride=1, padding=2)  # 5x5 conv, output 6 channels
        self.pool00 = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)  # 3x3 pool, stride2
        self.conv01 = nn.Conv2d(6, 12, kernel_size=4, stride=1, padding=1)  # 4x4 conv, output 12 channels
        self.pool01 = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)  # 3x3 pool, stride2

        # ----- Branch One (global) -----
        self.conv10 = nn.Conv2d(12, 20, kernel_size=5, stride=1, padding=2)  # 5x5 conv, output 20 channels
        self.pool10 = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)  # 3x3 pool, stride2
        # Residual blocks in branch one
        self.res11 = BasicResidualBlock(channels=20)
        self.pool11 = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)  # 3x3 pool, stride2
        self.res12 = BasicResidualBlock(channels=20)
        self.pool12 = nn.MaxPool2d(kernel_size=3, stride=1,
                                   padding=1)  # 3x3 pool, stride1 (no downsampling, just local pooling)
        self.res13 = BasicResidualBlock(channels=20)
        self.pool13 = nn.MaxPool2d(kernel_size=3, stride=1, padding=1)  # 3x3 pool, stride1

        # Fully connected layers for branch one
        # (The in_features for fc1 will be set in forward based on flattened feature size)
        self.fc1 = None  # placeholder, will initialize in forward once we know flatten dim
        self.fc2 = None  # placeholder for 10-d output from branch one

        # ----- Branch Two (local) -----
        self.conv20 = nn.Conv2d(12, 20, kernel_size=5, stride=1, padding=2)  # 5x5 conv, output 20 channels
        # Residual blocks in branch two (local feature extraction)
        self.res21 = BasicResidualBlock(channels=20)
        self.res22 = BasicResidualBlock(channels=20)
        self.res23 = BasicResidualBlock(channels=20)
        self.res24 = BasicResidualBlock(channels=20)
        self.conv21 = nn.Conv2d(20, 10, kernel_size=3, stride=1,
                                padding=1)  # 3x3 conv, output 10 channels (local value map)

        # Fully connected layer to combine branches and output actions
        self.fc3 = nn.Linear(10, num_actions)  # fc3 will take combined 10-d feature and output 8 actions
        self.dropout = nn.Dropout(p=0.5)  # dropout layer (used in fully connected stages)

    def forward(self, x1, x2=None, coords=None):
        """
        Forward pass for DB-CNN.
        :param x1: Tensor of shape (N, C1, H, W) for first input (e.g. environment image or combined channels).
        :param x2: Tensor of shape (N, C2, H, W) for second input (e.g. second image like target map), or None if not used.
        :param coords: Tensor of shape (N, 2) for start positions (optional, used to gather local branch outputs).
        :return: Tensor of shape (N, num_actions) logits for each action.
        """
        # If a second input is provided, concatenate it with the first along channel dimension
        if x2 is not None:
            x = torch.cat([x1, x2], dim=1)  # shape: (N, C1+C2, H, W)
        else:
            x = x1

        # Shared preprocessing
        x = F.relu(self.conv00(x))
        x = self.pool00(x)
        x = F.relu(self.conv01(x))
        x = self.pool01(x)
        # After preproc, shape is (N, 12, H/4, W/4) approximately.

        # Branch One (global features)
        b1 = F.relu(self.conv10(x))
        b1 = self.pool10(b1)
        b1 = self.res11(b1)
        b1 = self.pool11(b1)
        b1 = self.res12(b1)
        b1 = self.pool12(b1)
        b1 = self.res13(b1)
        b1 = self.pool13(b1)
        # Flatten branch one output
        b1_flat = b1.view(b1.size(0), -1)  # flatten to (N, feature_dim)
        # Initialize fully connected layers if not already (to adapt to any input size dynamically)
        if self.fc1 is None:
            # Set up fc1 and fc2 now that we know the input feature dimension
            self.fc1 = nn.Linear(b1_flat.size(1), 192)
            self.fc2 = nn.Linear(192, 10)
            # Move new layers to same device as existing model
            self.fc1.to(b1_flat.device)
            self.fc2.to(b1_flat.device)
        # Fully connected layers for branch one
        b1_out = F.relu(self.fc1(b1_flat))
        b1_out = self.dropout(b1_out)  # apply dropout in branch one fully-connected
        b1_out = self.fc2(b1_out)  # shape: (N, 10)

        # Branch Two (local features)
        b2 = F.relu(self.conv20(x))
        b2 = self.res21(b2)
        b2 = self.res22(b2)
        b2 = self.res23(b2)
        b2 = self.res24(b2)
        b2 = self.conv21(b2)  # shape: (N, 10, h, w) local value function map
        # If coordinates provided, gather local features at those coordinates
        if coords is not None:
            coords = coords.long()
            # Scale coordinates from original image scale to the scale of b2 feature map if necessary
            # Calculate scale factor between input and b2 feature map
            # (Assume input H->h_out = scale_factor, here roughly H/4 due to two stride2 pools in preprocessing)
            scale_y = b2.shape[2] / x1.shape[2]
            scale_x = b2.shape[3] / x1.shape[3]
            # Compute scaled coordinates (as integer indices)
            # Here we use floor (integer division) to map to nearest feature map index
            y_idx = (coords[:, 0].float() * scale_y).long().to(b2.device)
            x_idx = (coords[:, 1].float() * scale_x).long().to(b2.device)
            # Clamp coordinates to valid range
            y_idx = torch.clamp(y_idx, 0, b2.shape[2] - 1)
            x_idx = torch.clamp(x_idx, 0, b2.shape[3] - 1)
            # Gather local branch features at (y_idx, x_idx) for each sample
            N = b2.size(0)
            b2_out = b2[torch.arange(N, device=b2.device), :, y_idx, x_idx]  # shape: (N, 10)
        else:
            # If no coords, globally pool the local features (this scenario is uncommon)
            b2_out = F.adaptive_max_pool2d(b2, output_size=(1, 1)).view(b2.size(0), -1)  # (N, 10)

        # Combine branches: element-wise addition of the 10-dimensional feature vectors
        combined = b1_out + b2_out  # shape: (N, 10)
        combined = self.dropout(combined)  # apply dropout before final output if needed
        logits = self.fc3(combined)  # shape: (N, num_actions)
        return logits


class DCNN(nn.Module):
    """
    Deep Convolutional Neural Network (DCNN) baseline implementation.
    This is a single-branch CNN (similar to branch two of DB-CNN) used to estimate the value function or next action, without any residual connections.
    """

    def __init__(self, input_channels: int = 2, num_actions: int = 8):
        """
        :param input_channels: Number of input channels.
        :param num_actions: Number of output action classes.
        """
        super(DCNN, self).__init__()
        # Preprocessing layers (same as DB-CNN preprocessing)
        self.conv00 = nn.Conv2d(input_channels, 6, kernel_size=5, stride=1, padding=2)
        self.pool00 = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
        self.conv01 = nn.Conv2d(6, 12, kernel_size=4, stride=1, padding=1)
        self.pool01 = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
        # Convolutional layers for deep CNN (no skip connections, just sequential conv layers)
        self.conv20 = nn.Conv2d(12, 20, kernel_size=5, stride=1, padding=2)
        self.conv21 = nn.Conv2d(20, 20, kernel_size=3, stride=1, padding=1)
        self.conv22 = nn.Conv2d(20, 20, kernel_size=3, stride=1, padding=1)
        self.conv23 = nn.Conv2d(20, 20, kernel_size=3, stride=1, padding=1)
        self.conv24 = nn.Conv2d(20, 20, kernel_size=3, stride=1, padding=1)
        self.conv25 = nn.Conv2d(20, 10, kernel_size=3, stride=1,
                                padding=1)  # final conv to produce 10-channel feature map
        # Fully connected layer for output
        self.fc_out = nn.Linear(10, num_actions)
        self.dropout = nn.Dropout(p=0.5)

    def forward(self, x1, x2=None, coords=None):
        """
        Forward pass for DCNN baseline.
        :param x1: Input tensor (N, C1, H, W).
        :param x2: Second input tensor (N, C2, H, W), or None if not used.
        :param coords: Tensor of shape (N, 2) with start positions (optional).
        :return: Tensor of shape (N, num_actions) logits.
        """
        # Concatenate inputs if second input exists
        if x2 is not None:
            x = torch.cat([x1, x2], dim=1)
        else:
            x = x1
        # Preprocessing conv + pool
        x = F.relu(self.conv00(x))
        x = self.pool00(x)
        x = F.relu(self.conv01(x))
        x = self.pool01(x)
        # Convolutional sequence
        x = F.relu(self.conv20(x))
        x = F.relu(self.conv21(x))
        x = F.relu(self.conv22(x))
        x = F.relu(self.conv23(x))
        x = F.relu(self.conv24(x))
        x = self.conv25(x)  # shape: (N, 10, h, w)
        # If coordinates given, gather 10-d feature at that spatial location
        if coords is not None:
            coords = coords.long()
            # Scale coords from original image to feature map scale
            scale_y = x.shape[2] / x1.shape[2]
            scale_x = x.shape[3] / x1.shape[3]
            y_idx = (coords[:, 0].float() * scale_y).long().to(x.device)
            x_idx = (coords[:, 1].float() * scale_x).long().to(x.device)
            y_idx = torch.clamp(y_idx, 0, x.shape[2] - 1)
            x_idx = torch.clamp(x_idx, 0, x.shape[3] - 1)
            N = x.size(0)
            feat = x[torch.arange(N, device=x.device), :, y_idx, x_idx]  # shape: (N, 10)
        else:
            # Global max pool if no coords
            feat = F.adaptive_max_pool2d(x, output_size=(1, 1)).view(x.size(0), -1)  # (N, 10)
        feat = self.dropout(feat)
        logits = self.fc_out(feat)  # (N, num_actions)
        return logits