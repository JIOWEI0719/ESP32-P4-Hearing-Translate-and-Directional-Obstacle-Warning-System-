from pathlib import Path
import json

import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from torchvision import datasets, transforms


ROOT = Path(__file__).resolve().parent
SPLIT_DIR = ROOT / "dataset_split"
OUTPUT_DIR = ROOT / "outputs"
EXPORT_DIR = ROOT / "exports"
CHECKPOINT_PATH = OUTPUT_DIR / "gesture_cnn_224_best.pth"
ONNX_PATH = EXPORT_DIR / "gesture_cnn_224_gray.onnx"
IMG_SIZE = 224


class ConvBNAct(nn.Sequential):
    def __init__(self, in_ch, out_ch, kernel_size=3, stride=1, groups=1):
        padding = kernel_size // 2
        super().__init__(
            nn.Conv2d(
                in_ch,
                out_ch,
                kernel_size,
                stride,
                padding,
                groups=groups,
                bias=False,
            ),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
        )


class DSConv(nn.Module):
    def __init__(self, in_ch, out_ch, stride=1):
        super().__init__()
        self.block = nn.Sequential(
            ConvBNAct(in_ch, in_ch, kernel_size=3, stride=stride, groups=in_ch),
            ConvBNAct(in_ch, out_ch, kernel_size=1),
        )

    def forward(self, x):
        return self.block(x)


class TinyGestureCNN(nn.Module):
    def __init__(self, num_classes):
        super().__init__()
        self.features = nn.Sequential(
            ConvBNAct(1, 16, kernel_size=3, stride=2),
            DSConv(16, 24),
            DSConv(24, 32, stride=2),
            DSConv(32, 48),
            DSConv(48, 64, stride=2),
            DSConv(64, 96),
            DSConv(96, 128, stride=2),
            DSConv(128, 160),
            DSConv(160, 192, stride=2),
            DSConv(192, 192),
        )
        self.pool = nn.AdaptiveAvgPool2d(1)
        self.dropout = nn.Dropout(p=0.20)
        self.classifier = nn.Linear(192, num_classes)

    def forward(self, x):
        x = self.features(x)
        x = self.pool(x).flatten(1)
        x = self.dropout(x)
        return self.classifier(x)


def evaluate(model, dataset):
    loader = DataLoader(dataset, batch_size=16, shuffle=False, num_workers=0)
    confusion = torch.zeros(len(dataset.classes), len(dataset.classes), dtype=torch.int64)
    correct = 0
    total = 0

    with torch.no_grad():
        for images, labels in loader:
            predictions = model(images).argmax(dim=1)
            correct += (predictions == labels).sum().item()
            total += labels.numel()
            for true_label, prediction in zip(labels, predictions):
                confusion[true_label, prediction] += 1

    return correct / max(total, 1), confusion


def main():
    checkpoint = torch.load(CHECKPOINT_PATH, map_location="cpu", weights_only=False)
    classes = checkpoint["class_names"]
    class_to_idx = checkpoint["class_to_idx"]

    eval_transform = transforms.Compose(
        [
            transforms.Grayscale(num_output_channels=1),
            transforms.Resize((IMG_SIZE, IMG_SIZE)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5], std=[0.5]),
        ]
    )
    test_dataset = datasets.ImageFolder(SPLIT_DIR / "test", transform=eval_transform)
    if test_dataset.class_to_idx != class_to_idx:
        raise RuntimeError(
            f"Class mapping mismatch: checkpoint={class_to_idx}, dataset={test_dataset.class_to_idx}"
        )

    model = TinyGestureCNN(len(classes))
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    test_accuracy, confusion = evaluate(model, test_dataset)
    print(f"best_epoch: {checkpoint['epoch']}")
    print(f"best_val_acc: {checkpoint['val_acc']:.4f}")
    print(f"test_acc: {test_accuracy:.4f}")
    print("classes:", classes)
    print("confusion matrix rows=true cols=pred")
    print(confusion.numpy())

    EXPORT_DIR.mkdir(exist_ok=True)
    dummy = torch.randn(1, 1, IMG_SIZE, IMG_SIZE)
    torch.onnx.export(
        model,
        dummy,
        ONNX_PATH,
        export_params=True,
        opset_version=13,
        do_constant_folding=True,
        input_names=["input"],
        output_names=["logits"],
        dynamo=False,
    )

    (EXPORT_DIR / "labels.txt").write_text(
        "\n".join(classes) + "\n", encoding="utf-8"
    )
    (EXPORT_DIR / "class_to_idx.json").write_text(
        json.dumps(class_to_idx, indent=2), encoding="utf-8"
    )
    preprocess = {
        "input_shape": [1, 1, IMG_SIZE, IMG_SIZE],
        "color": "grayscale",
        "resize": [IMG_SIZE, IMG_SIZE],
        "normalization": "x = (pixel / 255.0 - 0.5) / 0.5",
        "training_range": [-1.0, 1.0],
        "classes": classes,
        "best_epoch": checkpoint["epoch"],
        "best_val_acc": checkpoint["val_acc"],
        "test_acc": test_accuracy,
        "confusion_matrix": confusion.tolist(),
    }
    (EXPORT_DIR / "preprocess.json").write_text(
        json.dumps(preprocess, indent=2), encoding="utf-8"
    )
    print("exported:", ONNX_PATH)


if __name__ == "__main__":
    main()
