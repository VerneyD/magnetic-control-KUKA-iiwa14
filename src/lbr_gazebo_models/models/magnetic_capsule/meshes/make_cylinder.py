from stl import mesh
import numpy as np

def create_cylinder(radius=0.01, height=0.05, segments=32):
    vertices = []
    faces = []

    # Création des points du cercle du bas et du haut
    for i in range(segments):
        angle = 2 * np.pi * i / segments
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        vertices.append([x, y, 0])        # Cercle bas (z=0)
        vertices.append([x, y, height])   # Cercle haut (z=height)

    # Centre bas et haut
    vertices.append([0, 0, 0])       # centre bas
    vertices.append([0, 0, height])  # centre haut

    # Faces latérales
    for i in range(segments):
        i1 = 2*i
        i2 = 2*((i+1) % segments)
        faces.append([i1, i2, i1+1])
        faces.append([i2, i2+1, i1+1])

    base_center = 2*segments
    top_center = 2*segments + 1

    # Faces bas
    for i in range(segments):
        i1 = 2*i
        i2 = 2*((i+1) % segments)
        faces.append([i1, i2, base_center])

    # Faces haut
    for i in range(segments):
        i1 = 2*i + 1
        i2 = 2*((i+1) % segments) + 1
        faces.append([i1, top_center, i2])

    vertices = np.array(vertices)
    faces = np.array(faces)

    cylinder_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, f in enumerate(faces):
        for j in range(3):
            cylinder_mesh.vectors[i][j] = vertices[f[j], :]

    return cylinder_mesh

if __name__ == "__main__":
    cyl = create_cylinder()
    cyl.save("cylinder.stl")
    print("Fichier cylinder.stl créé")
