import logging
import os
import time

import numpy as np
import rembg
import torch
from PIL import Image
import open3d as o3d

from tsr.system import TSR
from tsr.utils import remove_background, resize_foreground, save_video

class Timer:
    def __init__(self):
        self.items = {}
        self.time_scale = 1000.0  # ms
        self.time_unit = "ms"

    def start(self, name: str) -> None:
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        self.items[name] = time.time()
        logging.info(f"{name} ...")

    def end(self, name: str) -> float:
        if name not in self.items:
            return
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        start_time = self.items.pop(name)
        delta = time.time() - start_time
        t = delta * self.time_scale
        logging.info(f"{name} finished in {t:.2f}{self.time_unit}.")


class LRM_Reconstruction:

    def __init__(self, not_remove_bg=False):
        self.not_remove_bg = not_remove_bg

    def runner(self, image_path):
        
        timer = Timer()

        logging.basicConfig(
            format="%(asctime)s - %(levelname)s - %(message)s", level=logging.INFO
        )

        device = "cpu"
        # default="cuda:0"

        timer.start("Initializing model")
        model = TSR.from_pretrained(
            "stabilityai/TripoSR",
            config_name="config.yaml",
            weight_name="model.ckpt",
        )
        model.renderer.set_chunk_size(8192)
        model.to(device)
        timer.end("Initializing model")

        timer.start("Processing images")

        if not self.not_remove_bg:
            rembg_session = None
        else:
            rembg_session = rembg.new_session()

        if self.not_remove_bg:
            image = np.array(Image.open(image_path).convert("RGB"))
        else:
            image = remove_background(Image.open(image_path), rembg_session)
            image = resize_foreground(image, 0.85)
            image = np.array(image).astype(np.float32) / 255.0
            image = image[:, :, :3] * image[:, :, 3:4] + (1 - image[:, :, 3:4]) * 0.5
            image = Image.fromarray((image * 255.0).astype(np.uint8))
        timer.end("Processing images")

        logging.info(f"Running image...")

        timer.start("Running model")
        with torch.no_grad():
            scene_codes = model([image], device=device)
        timer.end("Running model")

        timer.start("Exporting mesh")
        meshes = model.extract_mesh(scene_codes, resolution=256)
        final_obj = meshes[0]
        o3d_mesh = o3d.geometry.TriangleMesh()
        o3d_mesh.vertices = o3d.utility.Vector3dVector(final_obj.vertices)
        o3d_mesh.triangles = o3d.utility.Vector3iVector(final_obj.faces)
        timer.end("Exporting mesh")

        return o3d_mesh

if __name__ == "__main__":

    def sorter(fileName):
        return int(str(fileName)[7:-4])

    output_dir = "/home/ivokosa/Desktop/Reconstruction_output/"
    run_str = "4"
    out_dir = output_dir + run_str + "/"
    prefix_len = len(out_dir) + 14

    path_suffix = run_str + "/images/"
    path_str = os.path.join(output_dir, path_suffix)
    files = os.listdir(path_str)
    filtered_files = [file for file in files if file.startswith("colour_")]
    filtered_files.sort(key=sorter)
    file_list = [os.path.join(path_str, i) for i in filtered_files]

    print("Running Length: ", str(len(file_list)))
    print(file_list)

    for i, file in enumerate(file_list):

        print("Run Number: ", str(i))

        run = LRM_Reconstruction()

        mesh_name = out_dir + "meshes/LRM_Mesh_" + str(file[prefix_len:-4]) + ".obj"
        mesh = run.runner(file)

        o3d.io.write_triangle_mesh(mesh_name, mesh)

        print("Sleeping on Iteration: ", str(i))

        time.sleep(15)
