#!/usr/bin/env python3
import torch
import os

MODEL_PATH = '/home/pi/Desktop/Intelligent-Vehicle-Navigation/AI/spatial_rl_model.pth'

print("🔍 Testing model file...")
print(f"Path: {MODEL_PATH}")
print(f"Exists: {os.path.exists(MODEL_PATH)}")

if os.path.exists(MODEL_PATH):
    print(f"Size: {os.path.getsize(MODEL_PATH)} bytes")
    
    try:
        checkpoint = torch.load(MODEL_PATH, map_location='cpu')
        print(f"✅ Loaded successfully")
        print(f"Type: {type(checkpoint)}")
        
        if isinstance(checkpoint, dict):
            print(f"Keys: {list(checkpoint.keys())}")
            for key, value in checkpoint.items():
                if hasattr(value, 'shape'):
                    print(f"  {key}: shape={value.shape}, dtype={value.dtype}")
                elif isinstance(value, dict):
                    print(f"  {key}: dict with {len(value)} items")
                    # Print first few keys of nested dict
                    for subkey in list(value.keys())[:3]:
                        print(f"    {subkey}: {type(value[subkey])}")
                else:
                    print(f"  {key}: {type(value)}")
        else:
            # Might be a state dict directly
            print(f"Checkpoint has {len(checkpoint)} items")
            print("First few keys:", list(checkpoint.keys())[:5] if hasattr(checkpoint, 'keys') else "N/A")
            
    except Exception as e:
        print(f"❌ Error loading: {e}")
        import traceback
        traceback.print_exc()
else:
    print("❌ File not found!")