import collada
from stl import mesh
import numpy as np

def dae_to_stl(input_dae, output_stl):
    # Load the .dae file
    dae_mesh = collada.Collada(input_dae)

    # Extract geometry data
    vertices = []
    faces = []

    for geometry in dae_mesh.geometries:
        for primitive in geometry.primitives:
            if isinstance(primitive, collada.triangleset.TriangleSet):
                vertices.extend(primitive.vertex)
                faces.extend(primitive.vertex_index)

    vertices = np.array(vertices)
    faces = np.array(faces)

    # Create the STL mesh
    stl_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, face in enumerate(faces):
        for j in range(3):
            stl_mesh.vectors[i][j] = vertices[face[j]]

    # Save to .stl file
    stl_mesh.save(output_stl)
    print(f"Converted {input_dae} to {output_stl}")
