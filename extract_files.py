import os
import shutil
import glob

def get_next_folder_number(experiment_folder):
    """Find the next available folder number for T folders in the given experiment folder"""
    existing_folders = glob.glob(os.path.join(experiment_folder, 'T*'))
    if not existing_folders:
        return 0
    
    numbers = []
    for folder in existing_folders:
        try:
            num = int(os.path.basename(folder)[1:])  # Extract number after 'T'
            numbers.append(num)
        except ValueError:
            continue
    
    return max(numbers) + 1 if numbers else 0

def main():
    # Files to process
    files_to_copy = [
        'MJDATA.TXT', 
        'mjmodel.mjb',
        'MUJOCO_LOG.TXT',
        'screenshot.png',
        'mjmodel.xml'
    ]
    
    model_dir = 'model'  # Directory containing the source files
    
    # Get experiment name from user
    experiment_name = input("Please enter a name for this experiment data: ")
    
    # Create data directory if it doesn't exist
    if not os.path.exists('data'):
        os.makedirs('data')
        print("Created 'data' directory")
    
    # Create experiment folder if it doesn't exist
    experiment_folder = os.path.join('data', experiment_name)
    if not os.path.exists(experiment_folder):
        os.makedirs(experiment_folder)
        print(f"Created experiment folder: {experiment_folder}")
    else:
        print(f"Using existing experiment folder: {experiment_folder}")
    
    # Get next T folder number specific to this experiment folder
    next_num = get_next_folder_number(experiment_folder)
    new_folder = os.path.join(experiment_folder, f'T{next_num}')
    os.makedirs(new_folder)
    print(f"Created folder: {new_folder}")
    
    # Copy files from model directory to new folder
    for file in files_to_copy:
        source_path = os.path.join(model_dir, file)
        if os.path.exists(source_path):
            shutil.copy2(source_path, os.path.join(new_folder, file))
            print(f"Copied {file} to {new_folder}")
        else:
            print(f"Warning: {file} not found in {model_dir}")
    
    # Delete original files from model directory except mjmodel.xml
    for file in files_to_copy:
        if file != 'mjmodel.xml':
            source_path = os.path.join(model_dir, file)
            if os.path.exists(source_path):
                try:
                    os.remove(source_path)
                    print(f"Deleted {file} from {model_dir}")
                except Exception as e:
                    print(f"Error deleting {file}: {e}")

if __name__ == "__main__":
    main()
