import shutil
import xml.etree.ElementTree as ET
import argparse
import sys
from pathlib import Path

from utils import dae_to_stl

def extract_mesh_paths(urdf_file):
    """
    Extract all mesh paths from a URDF file.
    
    Args:
        urdf_file (str): Path to the URDF file
        
    Returns:
        list: List of mesh paths found in the URDF file
    """
    try:
        # Parse the URDF file
        tree = ET.parse(urdf_file)
        root = tree.getroot()
        
        # Find all mesh elements in the URDF
        mesh_paths = []
        for mesh in root.findall('.//mesh'):
            filename = mesh.get('filename')
            if filename and filename.startswith('package://'):
                mesh_paths.append(filename)
        
        return mesh_paths
    
    except ET.ParseError as e:
        print(f"Error parsing URDF file: {e}", file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError:
        print(f"Error: URDF file '{urdf_file}' not found", file=sys.stderr)
        sys.exit(1)

def _remove_prefix(path):
    """
    Remove the prefix from a path.
    """
    return path.replace('package://', '')

def convert_dae_to_stl(dae_file, stl_file):
    """
    Convert a DAE file to an STL file.
    """
    dae_to_stl(dae_file, stl_file)

def copy_mesh(mesh_path, output_dir):
    """
    Copy a mesh to the output directory.
    """
    if mesh_path.endswith('.dae'):
        convert_dae_to_stl(mesh_path, Path(output_dir) / (str(Path(mesh_path).name) + '.stl'))
    else:
        shutil.copy(mesh_path, output_dir)

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Extract mesh paths from a URDF file')
    parser.add_argument('urdf_file', help='Path to the URDF file')
    parser.add_argument('output_dir', help='Path to the output directory')
    args = parser.parse_args()
    
    # Extract and print mesh paths
    mesh_paths = extract_mesh_paths(args.urdf_file)
    
    # Clean paths
    mesh_paths = [_remove_prefix(path) for path in mesh_paths]

    # Copy meshes to the output directory
    for path in mesh_paths:
        copy_mesh(path, args.output_dir)

    # Update the URDF file to reference the STL files instead of the DAE files
    with open(args.urdf_file, 'r') as file:
        urdf_content = file.read()
    urdf_content = urdf_content.replace('.dae', '.dae.stl')
    
    # Add '_processed' to the end of the file name in the output directory
    new_urdf_file = Path(args.output_dir) / (Path(args.urdf_file).stem + '_processed.urdf')
    with open(new_urdf_file, 'w') as file:
        file.write(urdf_content)

if __name__ == '__main__':
    main()
