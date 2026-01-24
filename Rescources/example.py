# Step 1: On your TRAINING computer (not the RPi), convert your model
import torch
import onnx
import onnxruntime as ort

# Load your trained model
checkpoint = torch.load("AI/checkpoints_working/working_final.pth", map_location='cpu')
model = checkpoint['model']
model.eval()  # Set to evaluation mode

# Create a dummy input matching your state size (e.g., [batch_size, state_dim])
dummy_input = torch.randn(1, 10)  # Adjust 10 to your actual state dimension

# Export to ONNX format
torch.onnx.export(
    model,
    dummy_input,
    "sac_agent_576.onnx",
    input_names=["state_input"],
    output_names=["action_output"],
    dynamic_axes={'state_input': {0: 'batch_size'}, 'action_output': {0: 'batch_size'}},
    opset_version=14
)

print("✅ Model converted to sac_agent_576.onnx")