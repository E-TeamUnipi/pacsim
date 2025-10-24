#XDG_SESSION_TYPE=x11 python3 visualize.py
import open3d as o3d
import os
import sys
import matplotlib.pyplot as plt

# Percorso del file .pcd generato dal programma C++
pcd_file_path = "cloud_test.pcd"

# Verifica che il file esista
if not os.path.exists(pcd_file_path):
    print(f"Errore: il file '{pcd_file_path}' non è stato trovato.")
    exit()

# 1. Leggi la point cloud dal file PCD
print("Lettura del file point cloud...")
pcd = o3d.io.read_point_cloud(pcd_file_path)

# Controlla se l'utente ha passato "STATIC" come argomento
if len(sys.argv) > 1 and sys.argv[1].upper() == "STATIC":
    print("Generazione immagine statica della point cloud...")

    # Crea un visualizzatore non interattivo
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    vis.add_geometry(pcd)
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

    # Salva screenshot
    image_path = "pointcloud.png"
    vis.capture_screen_image(image_path)
    vis.destroy_window()

    # Mostra l'immagine salvata
    img = plt.imread(image_path)
    plt.imshow(img)
    plt.axis("off")
    plt.show()

else:
    # 2. Visualizza la point cloud interattiva
    print("Visualizzazione interattiva della point cloud. Chiudi la finestra per terminare lo script.")
    o3d.visualization.draw_geometries([pcd])
