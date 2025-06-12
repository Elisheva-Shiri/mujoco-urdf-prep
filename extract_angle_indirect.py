import os
import pandas as pd
import numpy as np

# Define the column names for the QPOS data
qpos_columns = [
    'ra_shoulder_pan_joint', 'ra_shoulder_lift_joint', 'ra_elbow_joint', 
    'ra_wrist_1_joint', 'ra_wrist_2_joint', 'ra_wrist_3_joint', 
    'rh_WRJ2', 'rh_WRJ1', 'rh_FFJ4', 'rh_FFJ3', 'rh_FFJ2', 'rh_FFJ1', 
    'rh_MFJ4', 'rh_MFJ3', 'rh_MFJ2', 'rh_MFJ1', 'rh_RFJ4', 'rh_RFJ3', 
    'rh_RFJ2', 'rh_RFJ1', 'rh_LFJ5', 'rh_LFJ4', 'rh_LFJ3', 'rh_LFJ2', 
    'rh_LFJ1', 'rh_THJ5', 'rh_THJ4', 'rh_THJ3', 'rh_THJ2', 'rh_THJ1'
]

# List to store all data
all_data = []

# Get the current directory where the script is located
current_dir = os.path.dirname(os.path.abspath(__file__))
data_dir = os.path.join(current_dir, 'data')

print(f"Looking for data in: {data_dir}")

# Get all category folders
categories = [d for d in os.listdir(data_dir) if os.path.isdir(os.path.join(data_dir, d))]
print(f"Found categories: {categories}")

for category in categories:
    category_path = os.path.join(data_dir, category)
    print(f"\nProcessing category: {category}")
    
    # Get all run folders (T0, T1, etc.)
    runs = [d for d in os.listdir(category_path) if os.path.isdir(os.path.join(category_path, d))]
    print(f"Found runs: {runs}")
    
    for run in runs:
        run_path = os.path.join(category_path, run)
        mjdata_path = os.path.join(run_path, 'MJDATA.TXT')
        
        if not os.path.exists(mjdata_path):
            print(f"Warning: MJDATA.TXT not found in {run}")
            continue
            
        print(f"Processing run: {run}")
        
        try:
            with open(mjdata_path, 'r') as f:
                content = f.read()
                
            # Find the QPOS section
            if 'QPOS' in content:
                # Get everything after QPOS until the next section or end of file
                qpos_section = content.split('QPOS')[1].split('\n\n')[0]
                
                # Extract all numbers from the QPOS section
                qpos_values = []
                for line in qpos_section.split('\n'):
                    line = line.strip()
                    if line:  # Skip empty lines
                        try:
                            value = float(line)
                            qpos_values.append(value)
                        except ValueError:
                            continue
                
                # Verify we have the correct number of values
                if len(qpos_values) == len(qpos_columns):
                    # Create a row of data
                    row_data = {
                        'category': category,
                        'run': run
                    }
                    # Add the QPOS values
                    for col, val in zip(qpos_columns, qpos_values):
                        row_data[col] = val
                    
                    all_data.append(row_data)
                    print(f"Successfully processed {run} - Found {len(qpos_values)} values")
                else:
                    print(f"Warning: Expected {len(qpos_columns)} values but got {len(qpos_values)} in {run}")
            else:
                print(f"Warning: QPOS section not found in {run}")
        except Exception as e:
            print(f"Error processing {run}: {str(e)}")
            continue

# Create DataFrame and save to CSV
if all_data:
    df = pd.DataFrame(all_data)
    output_path = os.path.join(data_dir, 'data_angles.csv')
    df.to_csv(output_path, index=False)
    print(f"\nData successfully saved to {output_path}")
    print(f"Total entries processed: {len(all_data)}")
    print("\nCategories processed:", categories)
else:
    print("\nNo data was found to process")
