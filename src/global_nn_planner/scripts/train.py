import time
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from torch.utils.tensorboard import SummaryWriter
import joblib
import numpy as np
from model import VIN, DBCNN, DCNN
from pathlib import Path
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
import torchvision
import torchvision.transforms as transforms
from PIL import Image

class PathPlanningDataset(Dataset):
    def __init__(self, X1, Y, X2=None, coords=None):
        self.X1 = X1
        self.Y = Y
        self.X2 = X2
        self.coords = coords
        self.has_x2 = X2 is not None
        self.has_coords = coords is not None

    def __len__(self):
        return self.Y.shape[0]

    def __getitem__(self, idx):
        if self.has_x2 and self.has_coords:
            return self.X1[idx], self.X2[idx], self.coords[idx], self.Y[idx]
        elif self.has_x2:
            return self.X1[idx], self.X2[idx], self.Y[idx]
        elif self.has_coords:
            return self.X1[idx], self.coords[idx], self.Y[idx]
        else:
            return self.X1[idx], self.Y[idx]

def load_data(data_path):
    with open(data_path, 'rb') as f:
        data = joblib.load(f)

    if isinstance(data, (tuple, list)) and len(data) == 4:
        train_x1, train_x2, train_coords, train_y = data
        test_x1, test_x2, test_coords, test_y = train_x1, train_x2, train_coords, train_y
    else:
        raise ValueError("Unsupported data format")

    def to_tensor_and_channels(arr):
        if arr is None:
            return None
        if not torch.is_tensor(arr):
            if isinstance(arr, list) and isinstance(arr[0], np.ndarray):
                arr = np.array(arr)
            arr = torch.tensor(arr)
        if arr.dtype != torch.float32:
            arr = arr.float()
        if arr.ndim == 3:
            arr = arr.unsqueeze(1)
        elif arr.ndim == 4 and arr.shape[1] > 12:
            arr = arr.permute(0, 3, 1, 2)
        return arr

    train_x1 = to_tensor_and_channels(train_x1)
    train_x2 = to_tensor_and_channels(train_x2)
    test_x1 = to_tensor_and_channels(test_x1)
    test_x2 = to_tensor_and_channels(test_x2)

    train_y = torch.tensor(train_y).long()
    test_y = torch.tensor(test_y).long()
    train_coords = torch.tensor(train_coords) if train_coords is not None else None
    test_coords = torch.tensor(test_coords) if test_coords is not None else None

    train_dataset = PathPlanningDataset(train_x1, train_y, X2=train_x2, coords=train_coords)
    test_dataset = PathPlanningDataset(test_x1, test_y, X2=test_x2, coords=test_coords)
    return train_dataset, test_dataset

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Train VIN/DB-CNN/DCNN models in PyTorch")
    parser.add_argument("--model", type=str, choices=["VIN", "DBCNN", "DCNN"], default="DCNN")
    parser.add_argument("--data", type=str, default="/home/gr-agv-x9xy/Downloads/pretraining_dataset/db_cnn_data/data_64.pkl")
    parser.add_argument("--epochs", type=int, default=3000)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--learning-rate", type=float, default=0.0001)
    parser.add_argument("--logdir", type=str, default="runs")
    args = parser.parse_args()

    train_dataset, test_dataset = load_data(args.data)
    train_loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True)
    test_loader = DataLoader(test_dataset, batch_size=args.batch_size, shuffle=False)

    in_channels = train_dataset.X1.shape[1]
    if train_dataset.has_x2:
        in_channels += train_dataset.X2.shape[1]

    if args.model == "VIN":
        model = VIN(input_channels=in_channels, num_actions=8)
    elif args.model == "DBCNN":
        model = DBCNN(input_channels=in_channels, num_actions=8)
    else:
        model = DCNN(input_channels=in_channels, num_actions=8)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate)
    criterion = nn.CrossEntropyLoss()
    logname = f"{args.model}_{time.strftime('%Y%m%d-%H%M%S')}"
    logdir = Path(args.logdir) / logname
    writer = SummaryWriter(log_dir=logdir)

    for epoch in range(1, args.epochs + 1):
        model.train()
        epoch_loss, correct, total = 0.0, 0, 0
        start_time = time.time()

        for batch in train_loader:
            batch = [b.to(device) for b in batch]
            if train_dataset.has_x2 and train_dataset.has_coords:
                X1, X2, coords, Y = batch
                outputs = model(X1, X2, coords)
            elif train_dataset.has_x2:
                X1, X2, Y = batch
                outputs = model(X1, X2)
            elif train_dataset.has_coords:
                X1, coords, Y = batch
                outputs = model(X1, None, coords)
            else:
                X, Y = batch
                outputs = model(X)

            loss = criterion(outputs, Y)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            epoch_loss += loss.item() * Y.size(0)
            _, predicted = torch.max(outputs, 1)
            correct += (predicted == Y).sum().item()
            total += Y.size(0)

        avg_loss = epoch_loss / total
        train_acc = correct / total

        model.eval()
        test_correct, test_total = 0, 0
        all_preds, all_labels = [], []
        with torch.no_grad():
            for batch in test_loader:
                batch = [b.to(device) for b in batch]
                if test_dataset.has_x2 and test_dataset.has_coords:
                    X1, X2, coords, Y = batch
                    outputs = model(X1, X2, coords)
                elif test_dataset.has_x2:
                    X1, X2, Y = batch
                    outputs = model(X1, X2)
                elif test_dataset.has_coords:
                    X1, coords, Y = batch
                    outputs = model(X1, None, coords)
                else:
                    X, Y = batch
                    outputs = model(X)
                _, predicted = torch.max(outputs, 1)
                test_correct += (predicted == Y).sum().item()
                test_total += Y.size(0)
                all_preds.extend(predicted.cpu().numpy())
                all_labels.extend(Y.cpu().numpy())

        test_acc = test_correct / test_total

        cm = confusion_matrix(all_labels, all_preds)
        disp = ConfusionMatrixDisplay(confusion_matrix=cm)
        disp.plot(cmap="Blues")
        plt.title(f"Confusion Matrix at Epoch {epoch}")
        cm_path = f"{logdir}/confusion_matrix_epoch_{epoch}.png"
        plt.savefig(cm_path)
        plt.close()

        img = Image.open(cm_path).convert("RGB")
        img_tensor = transforms.ToTensor()(img)
        writer.add_image("Confusion_Matrix", img_tensor, global_step=epoch)

        writer.add_scalar("Loss/Train", avg_loss, epoch)
        writer.add_scalar("Accuracy/Train", train_acc, epoch)
        writer.add_scalar("Accuracy/Test", test_acc, epoch)

        print(f"Epoch {epoch}/{args.epochs}: Train Loss = {avg_loss:.4f}, Train Acc = {train_acc:.4f}, Test Acc = {test_acc:.4f}, Time = {time.time() - start_time:.2f}s")

    torch.save(model.state_dict(), logdir / f"{args.model}_final.pth")
    print(f"Saved model to {logdir / f'{args.model}_final.pth'}")
    writer.close()
