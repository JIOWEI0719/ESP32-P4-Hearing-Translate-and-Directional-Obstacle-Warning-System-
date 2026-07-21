from pathlib import Path
import argparse
import json

import torch
from torch.utils.data import DataLoader, ConcatDataset
from torchvision import datasets, transforms


ROOT = Path(__file__).resolve().parent
EXPORT_DIR = ROOT / "exports"
SPLIT_DIR = ROOT / "dataset_split"
SOURCE_DIR = ROOT / "gesture_dataset_fisheye_224"

ONNX_MODEL_PATH = EXPORT_DIR / "gesture_cnn_224_gray.onnx"
ESPDL_MODEL_PATH = EXPORT_DIR / "gesture_cnn_224_gray_esp32p4.espdl"
INPUT_SHAPE = [1, 1, 224, 224]
TARGET = "esp32p4"
NUM_OF_BITS = 8


def parse_args():
    parser = argparse.ArgumentParser(
        description="Quantize the 224x224 grayscale gesture ONNX model to ESP-DL .espdl."
    )
    parser.add_argument("--device", default="cpu", choices=["cpu", "cuda"])
    parser.add_argument("--calib-steps", type=int, default=32)
    parser.add_argument("--calib-batch-size", type=int, default=1)
    parser.add_argument("--eval-batch-size", type=int, default=1)
    parser.add_argument("--skip-eval", action="store_true")
    parser.add_argument("--skip-export", action="store_true")
    return parser.parse_args()


def get_image_root():
    if (SPLIT_DIR / "train").exists() and (SPLIT_DIR / "val").exists():
        return SPLIT_DIR
    if SOURCE_DIR.exists():
        return SOURCE_DIR
    raise FileNotFoundError(
        "Neither dataset_split nor gesture_dataset_fisheye_224 was found."
    )


def build_transform():
    return transforms.Compose(
        [
            transforms.Grayscale(num_output_channels=1),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5], std=[0.5]),
        ]
    )


def build_datasets():
    tfm = build_transform()
    image_root = get_image_root()

    if image_root == SPLIT_DIR:
        train_ds = datasets.ImageFolder(SPLIT_DIR / "train", transform=tfm)
        val_ds = datasets.ImageFolder(SPLIT_DIR / "val", transform=tfm)
        test_root = SPLIT_DIR / "test"
        test_ds = datasets.ImageFolder(test_root, transform=tfm) if test_root.exists() else val_ds
        calib_ds = ConcatDataset([train_ds, val_ds])
        eval_ds = test_ds
        classes = train_ds.classes
        class_to_idx = train_ds.class_to_idx
    else:
        all_ds = datasets.ImageFolder(SOURCE_DIR, transform=tfm)
        calib_ds = all_ds
        eval_ds = all_ds
        classes = all_ds.classes
        class_to_idx = all_ds.class_to_idx

    return calib_ds, eval_ds, classes, class_to_idx


def collate_for_quant(batch):
    # ESP-PPQ calls this function on each item produced by the dataloader.
    # With the default PyTorch collate this is usually (images, labels).
    if isinstance(batch, torch.Tensor):
        return batch.to(CURRENT_DEVICE)
    if isinstance(batch, (tuple, list)) and len(batch) == 2 and isinstance(batch[0], torch.Tensor):
        return batch[0].to(CURRENT_DEVICE)
    images = torch.stack([item[0] for item in batch], dim=0)
    return images.to(CURRENT_DEVICE)


def accuracy_from_logits(logits, labels):
    preds = logits.argmax(dim=1)
    return (preds == labels).sum().item(), labels.numel()


def evaluate_quantized_graph(graph, eval_loader, device):
    from esp_ppq.executor.torch import TorchExecutor

    executor = TorchExecutor(graph=graph, device=device)
    correct = 0
    total = 0
    confusion = None

    for images, labels in eval_loader:
        images = images.to(device)
        labels = labels.to(device)
        outputs = executor(images)
        logits = outputs[0]

        batch_correct, batch_total = accuracy_from_logits(logits, labels)
        correct += batch_correct
        total += batch_total

        preds = logits.argmax(dim=1).detach().cpu()
        cpu_labels = labels.detach().cpu()
        class_count = logits.shape[1]
        if confusion is None:
            confusion = torch.zeros(class_count, class_count, dtype=torch.int64)
        for true_label, pred_label in zip(cpu_labels, preds):
            confusion[true_label, pred_label] += 1

    return correct / max(total, 1), confusion


def maybe_rewrite_onnx_to_single_file():
    # PyTorch may export an external .onnx.data file. Most tools can read it, but
    # a single-file ONNX is easier to move and debug. If onnx is available, create one.
    single_file = EXPORT_DIR / "gesture_cnn_224_gray_single.onnx"

    try:
        import onnx
        from onnx import external_data_helper
    except ImportError:
        return ONNX_MODEL_PATH

    model = onnx.load(str(ONNX_MODEL_PATH), load_external_data=True)
    external_data_helper.convert_model_from_external_data(model)
    onnx.save_model(model, str(single_file), save_as_external_data=False)
    print(f"Created single-file ONNX: {single_file}")
    return single_file


def main():
    args = parse_args()

    global CURRENT_DEVICE
    CURRENT_DEVICE = args.device
    if CURRENT_DEVICE == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested, but torch.cuda.is_available() is false.")

    if not ONNX_MODEL_PATH.exists():
        raise FileNotFoundError(f"Missing ONNX model: {ONNX_MODEL_PATH}")

    calib_ds, eval_ds, classes, class_to_idx = build_datasets()
    calib_loader = DataLoader(
        calib_ds,
        batch_size=args.calib_batch_size,
        shuffle=False,
        num_workers=0,
    )
    eval_loader = DataLoader(
        eval_ds,
        batch_size=args.eval_batch_size,
        shuffle=False,
        num_workers=0,
    )

    print("classes:", classes)
    print("class_to_idx:", class_to_idx)
    print("calibration samples:", len(calib_ds))
    print("evaluation samples:", len(eval_ds))
    print("target:", TARGET)
    print("input_shape:", INPUT_SHAPE)
    print("device:", CURRENT_DEVICE)

    onnx_path = maybe_rewrite_onnx_to_single_file()

    from esp_ppq.api import espdl_quantize_onnx

    quant_graph = espdl_quantize_onnx(
        onnx_import_file=str(onnx_path),
        espdl_export_file=str(ESPDL_MODEL_PATH),
        calib_dataloader=calib_loader,
        calib_steps=args.calib_steps,
        input_shape=INPUT_SHAPE,
        inputs=None,
        target=TARGET,
        num_of_bits=NUM_OF_BITS,
        collate_fn=collate_for_quant,
        dispatching_override=None,
        device=CURRENT_DEVICE,
        error_report=True,
        skip_export=args.skip_export,
        export_test_values=True,
        verbose=1,
    )

    metadata = {
        "target": TARGET,
        "num_of_bits": NUM_OF_BITS,
        "input_shape": INPUT_SHAPE,
        "classes": classes,
        "class_to_idx": class_to_idx,
        "preprocess": {
            "color": "grayscale",
            "resize": [224, 224],
            "normalization": "x = (pixel / 255.0 - 0.5) / 0.5",
        },
        "espdl_model": str(ESPDL_MODEL_PATH),
    }

    if not args.skip_eval:
        quant_acc, confusion = evaluate_quantized_graph(quant_graph, eval_loader, CURRENT_DEVICE)
        metadata["quantized_eval_acc"] = quant_acc
        print(f"quantized_eval_acc: {quant_acc:.4f}")
        if confusion is not None:
            print("confusion matrix rows=true cols=pred")
            print(confusion.numpy())
            metadata["confusion_matrix"] = confusion.numpy().tolist()

    metadata_path = EXPORT_DIR / "gesture_cnn_224_gray_esp32p4_metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print("metadata:", metadata_path)
    if not args.skip_export:
        print("espdl:", ESPDL_MODEL_PATH)
        print("info :", ESPDL_MODEL_PATH.with_suffix(".info"))
        print("json :", ESPDL_MODEL_PATH.with_suffix(".json"))


if __name__ == "__main__":
    CURRENT_DEVICE = "cpu"
    main()
