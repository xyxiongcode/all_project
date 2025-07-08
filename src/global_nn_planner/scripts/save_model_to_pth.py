import os
from model import VIN, DBCNN, DCNN
import torch

ckpt_dir = "/home/gr-agv-x9xy/isaac_sim_ws/src/global_nn_planner/scripts/ckpt"
output_dir = "/home/gr-agv-x9xy/isaac_sim_ws/src/global_nn_planner/scripts/model_pths"
os.makedirs(output_dir, exist_ok=True)

for file in os.listdir(ckpt_dir):
    if file.endswith(".ckpt"):
        print(f"🔍 Found ckpt file: {file}")
        ckpt_path = os.path.join(ckpt_dir, file)
        ckpt = torch.load(ckpt_path, map_location='cpu')
        print(f"✅ Loaded keys: {list(ckpt.keys())}")

        state_dict = ckpt['state_dict'] if 'state_dict' in ckpt else ckpt
        keys = list(state_dict.keys())

        if 'conv_r.weight' in keys:
            model = VIN(input_channels=2, num_actions=8)
            model_type = 'VIN'
        elif 'fc1.weight' in keys:
            model = DBCNN(input_channels=2, num_actions=8)
            model_type = 'DBCNN'
        elif 'fc_out.weight' in keys:
            model = DCNN(input_channels=2, num_actions=8)
            model_type = 'DCNN'
        else:
            print(f"❌ Cannot identify model type for {file}")
            continue

        new_state_dict = {k.replace('model.', '').replace('net.', ''): v for k, v in state_dict.items()}

        try:
            model.load_state_dict(new_state_dict, strict=False)
            print(f"✅ Loaded weights for {model_type}")
        except Exception as e:
            print(f"❌ Failed to load weights for {model_type}: {e}")
            continue

        output_path = os.path.join(output_dir, f"{model_type}_{file.replace('.ckpt', '.pth')}")
        torch.save(model.state_dict(), output_path)
        print(f"📦 Saved to: {output_path}")



