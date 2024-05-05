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

    def __init__(self, not_remove_bg=False, render=False):
        self.not_remove_bg = not_remove_bg
        self.render = render

    def runner(self, img_lst, out_dir):
        
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

        images = []

        if not self.not_remove_bg:
            rembg_session = None
        else:
            rembg_session = rembg.new_session()

        for i, image_path in enumerate(img_lst):
            if self.not_remove_bg:
                image = np.array(Image.open(image_path).convert("RGB"))
            else:
                image = remove_background(Image.open(image_path), rembg_session)
                image = resize_foreground(image, 0.85)
                image = np.array(image).astype(np.float32) / 255.0
                image = image[:, :, :3] * image[:, :, 3:4] + (1 - image[:, :, 3:4]) * 0.5
                image = Image.fromarray((image * 255.0).astype(np.uint8))
            images.append(image)
        timer.end("Processing images")

        for i, image in enumerate(images):
            logging.info(f"Running image {i + 1}/{len(images)} ...")

            timer.start("Running model")
            with torch.no_grad():
                scene_codes = model([image], device=device)
            timer.end("Running model")

            if self.render:
                timer.start("Rendering")
                render_images = model.render(scene_codes, n_views=30, return_type="pil")
                for ri, render_image in enumerate(render_images[0]):
                    render_image.save(os.path.join(str(i), f"render_{ri:03d}.png"))
                save_video(
                    render_images[0], os.path.join(str(i), f"render.mp4"), fps=30
                )
                timer.end("Rendering")

            timer.start("Exporting mesh")
            meshes = model.extract_mesh(scene_codes, resolution=256)
            final_obj = meshes[0]
            o3d_mesh = o3d.geometry.TriangleMesh()
            o3d_mesh.vertices = o3d.utility.Vector3dVector(final_obj.vertices)
            o3d_mesh.triangles = o3d.utility.Vector3iVector(final_obj.faces)

            mesh_name = out_dir + "meshes/tripoMesh_" + str(i) + ".obj"
            o3d.io.write_triangle_mesh(mesh_name, o3d_mesh)

            timer.end("Exporting mesh")
            # return o3d_mesh

if __name__ == "__main__":

    output_dir = "/home/ivokosa/Desktop/Results/"
    run_str = "Teapot_3_Views"
    path_suffix = run_str + "/images/"

    path_str = os.path.join(output_dir, path_suffix)

    files = os.listdir(path_str)
    filtered_files = [file for file in files if file.startswith("colour_")]

    filtered_files.sort()

    file_list = [os.path.join(path_str, i) for i in filtered_files]

    run = LRM_Reconstruction()

    mesh = o3d.geometry.TriangleMesh()

    out_dir = output_dir + run_str + "/"

    mesh = run.runner(file_list, out_dir)
