#!/usr/bin/env python3
import re
import sys
import cv2
import numpy as np
from pathlib import Path
from urllib.parse import quote

import requests
from rich.console import Console
from rich.prompt import Prompt, Confirm
from rich.table import Table
from rich.progress import Progress, SpinnerColumn, TextColumn, BarColumn, TimeElapsedColumn
import imageio  # Only used for GIF

DEFAULT_ESP32_IP = "192.168.43.133"
DEFAULT_FPS = 10
DEFAULT_OUTPUT_DIR = "videos"
SUPPORTED_FORMATS = {
    "mp4": "MP4 (H.264)",
    "avi": "AVI (Motion JPEG)",
    "mov": "QuickTime MOV",
    "gif": "Animated GIF (slower, larger)"
}

console = Console()

def get_user_settings():
    console.rule("[bold cyan]Configure Settings")

    # ESP32 IP
    esp32_ip = Prompt.ask("   ESP32 IP Address", default=DEFAULT_ESP32_IP)

    # FPS
    fps = Prompt.ask("   Video FPS", default=str(DEFAULT_FPS))
    try:
        fps = int(fps)
        if fps < 1:
            fps = DEFAULT_FPS
    except ValueError:
        fps = DEFAULT_FPS

    # Save images?
    save_images = Confirm.ask("   Save images to disk?", default=True)

    # Output format
    console.print("\n   [bold]Select output format:[/]")
    fmt_table = Table(show_header=False, box=None, padding=(0, 2))
    for i, (ext, desc) in enumerate(SUPPORTED_FORMATS.items(), 1):
        default_mark = " [dim](default)" if ext == "mp4" else ""
        fmt_table.add_row(f"[dim]{i}.[/]", f"[green]{desc}[/]{default_mark}")
    console.print(fmt_table, justify="center")

    fmt_choice = Prompt.ask(
        f"   Format [1–{len(SUPPORTED_FORMATS)}]",
        choices=[str(i) for i in range(1, len(SUPPORTED_FORMATS) + 1)],
        default="1"
    )
    output_format = list(SUPPORTED_FORMATS.keys())[int(fmt_choice) - 1]

    console.print()
    return {
        "esp32_ip": esp32_ip,
        "fps": fps,
        "save_images": save_images,
        "output_format": output_format
    }

def get_scan_folders(base_url, timeout=10):
    console.rule("[bold cyan]Fetching Scan Folders")
    try:
        resp = requests.get(f"{base_url}/files", timeout=timeout)
        resp.raise_for_status()
        data = resp.json()
        folders = [f for f in data.get("folders", []) if re.match(r"^scan_\d+$", f)]
        folders = ["/" + f for f in folders]
        folders.sort(key=lambda x: int(x.split('_')[1]))
        return folders
    except Exception as e:
        console.print(f"[red]❌ Failed to fetch folders: {e}")
        return []

def select_folder(folders):
    if not folders:
        console.print("[yellow]⚠️ No scan folders found.")
        return None

    console.print("\n   [bold]Available scan folders:[/]")
    folder_table = Table(show_header=False, box=None, padding=(0, 2))
    for i, f in enumerate(folders, 1):
        folder_table.add_row(f"[dim]{i}.[/]", f"[green]{f}[/]")
    console.print(folder_table, justify="center")

    choices = [str(i) for i in range(1, len(folders) + 1)]
    choice = Prompt.ask(f"   Select folder [1–{len(folders)}]", choices=choices, default="1")
    console.print()
    return folders[int(choice) - 1]

def fetch_image_list(base_url, folder, timeout=10):
    console.rule(f"[bold cyan]Fetching Image List from {folder}")
    try:
        resp = requests.get(f"{base_url}/files?dir={quote(folder)}", timeout=timeout)
        resp.raise_for_status()
        data = resp.json()
        relative_files = [f for f in data.get("files", []) if f.endswith(".jpg")]
        if not relative_files:
            console.print(f"[red]❌ No images found in {folder}")
            return []
        full_paths = [f"{folder}/{f}" for f in relative_files]

        def sort_key(p):
            match = re.search(r'img_(\d+)', p)
            return int(match.group(1)) if match else 0
        full_paths.sort(key=sort_key)
        console.print(f"   ✅ Found {len(full_paths)} images\n")
        return full_paths
    except Exception as e:
        console.print(f"[red]❌ Failed to list images: {e}\n")
        return []

def download_images(base_url, image_paths, save_to_disk, folder_name, timeout=10):
    # console.rule("[bold cyan]Downloading Images")
    frames = []
    out_dir = Path("downloads") / folder_name.lstrip("/") if save_to_disk else None
    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)

    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        BarColumn(bar_width=None),
        TextColumn("[progress.percentage]{task.percentage:>3.0f}%"),
        TimeElapsedColumn(),
        console=console,
    ) as progress:
        task = progress.add_task("   Downloading", total=len(image_paths))
        for full_path in image_paths:
            url = f"{base_url}/download?file={quote(full_path)}"
            try:
                resp = requests.get(url, timeout=timeout)
                resp.raise_for_status()
                arr = np.frombuffer(resp.content, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    frames.append(frame)
                    if out_dir:
                        (out_dir / Path(full_path).name).write_bytes(resp.content)
            except Exception as e:
                console.print(f"[red]⚠️ Failed: {full_path}")
            progress.advance(task)

    if save_to_disk and out_dir:
        console.print(f"   ✅ Saved images to: [dim]downloads/{folder_name.lstrip('/')}/[/]\n")
    console.print(f"\n   ✅ Loaded {len(frames)} frames into memory\n")
    return frames

def save_video(frames, output_path, fps, fmt):
    if not frames:
        return False
    console.rule("[bold cyan]Encoding Video")
    h, w = frames[0].shape[:2]

    output_path.parent.mkdir(parents=True, exist_ok=True)

    if fmt == "gif":
        # GIF: use imageio
        with imageio.get_writer(str(output_path), mode='I', fps=fps) as writer:
            with Progress(
                SpinnerColumn(),
                TextColumn("[progress.description]{task.description}"),
                BarColumn(bar_width=None),
                TimeElapsedColumn(),
                console=console,
            ) as progress:
                task = progress.add_task("   Encoding GIF", total=len(frames))
                for frame in frames:
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    writer.append_data(rgb)
                    progress.advance(task)
        return True

    else:
        # Video: use OpenCV
        codec_map = {
            "mp4": "mp4v",
            "avi": "MJPG",
            "mov": "mp4v"
        }
        fourcc = cv2.VideoWriter_fourcc(*codec_map[fmt])
        out = cv2.VideoWriter(str(output_path), fourcc, fps, (w, h))
        if not out.isOpened():
            console.print("[red]❌ Video writer failed\n")
            return False

        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            BarColumn(bar_width=None),
            TimeElapsedColumn(),
            console=console,
        ) as progress:
            task = progress.add_task("   Encoding", total=len(frames))
            for frame in frames:
                out.write(frame)
                progress.advance(task)
        out.release()
        return True

def main():
    console.clear()
    console.print("[bold green]📸 ESP32-CAM Scan to Video", justify="center")
    console.print()

    settings = get_user_settings()
    base_url = f"http://{settings['esp32_ip']}"

    folders = get_scan_folders(base_url)
    if not folders:
        sys.exit(1)

    selected = select_folder(folders)
    if not selected:
        sys.exit(0)

    if not Confirm.ask(f"   Proceed with [cyan]{selected}[/]?", default=True):
        console.print("\n[dim]Cancelled.")
        return

    image_paths = fetch_image_list(base_url, selected)
    if not image_paths:
        sys.exit(1)

    frames = download_images(base_url, image_paths, settings["save_images"], selected)
    if not frames:
        console.print("[red]❌ No valid frames to encode.\n")
        sys.exit(1)

    # Build default filename with chosen format
    base_name = selected.lstrip('/')
    default_file = f"{base_name}.{settings['output_format']}"
    default_path = Path(DEFAULT_OUTPUT_DIR) / default_file

    output_path_str = Prompt.ask("   Output path", default=str(default_path))
    output_path = Path(output_path_str)

    success = save_video(frames, output_path, settings["fps"], settings["output_format"])
    if success:
        console.print(f"\n[bold green]🎉 Done! Saved: [cyan]{output_path.absolute()}[/]")
        if settings["save_images"]:
            console.print(f"[dim]📁 Images saved in: downloads/{selected.lstrip('/')}/")
    else:
        console.print("[red]💥 Encoding failed.\n")
        sys.exit(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        console.print("\n[yellow]⚠️ Interrupted by user.")
        sys.exit(0)
    except Exception as e:
        console.print(f"\n[red]💥 Unexpected error: {e}")
        sys.exit(1)
