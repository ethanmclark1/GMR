import pickle
import numpy as np

def inspect_pkl(pkl_file):
    """Inspect the structure of the PKL file"""
    print("="*60)
    print("PKL FILE INSPECTION")
    print("="*60)
    
    with open(pkl_file, 'rb') as f:
        motion_data = pickle.load(f)
    
    print("\n1. PKL Structure:")
    print(f"   Type: {type(motion_data)}")
    
    if isinstance(motion_data, dict):
        print(f"   Keys: {list(motion_data.keys())}")
        print("\n2. Data shapes:")
        for key, value in motion_data.items():
            if isinstance(value, np.ndarray):
                print(f"   {key}: shape={value.shape}, dtype={value.dtype}")
            else:
                print(f"   {key}: type={type(value)}")
        
        print("\n3. First frame sample:")
        for key, value in motion_data.items():
            if isinstance(value, np.ndarray) and len(value) > 0:
                print(f"   {key}[0]: {value[0]}")
    
    return motion_data

if __name__ == "__main__":
    data = inspect_pkl('motion_data/walk1_subject1.pkl')