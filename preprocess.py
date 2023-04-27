import os
import sys
import glob
import subprocess
from configs.create_config import createRtabmapConfig

if __name__ == '__main__':

    PROCESS_MKV_PATH = "preprocess/bin/preprocess_mkv.exe"
    
    if len(sys.argv) != 2:
        print("Invalid number of input argumens!")
        print("Usage: python preprocess.py <dataset_dir>")
        print("<dataset_dir> should include 'mkv' directory")
        print("that contains .mkv files.")
        sys.exit()
        
    # Check that extract_mkv.exe is found
    if not os.path.exists(PROCESS_MKV_PATH):
        print("preprocess_mkv.exe not found!")
        print("PROCESS_MKV_PATH is currently")
        print(PROCESS_MKV_PATH)
        sys.exit()
        
    # Check if mkv directory exists
    dataset_dir = sys.argv[1]
    mkv_dir = os.path.join(dataset_dir, "mkv")
    if os.path.isdir(mkv_dir):
        mkv_paths = glob.glob(mkv_dir + '/*.mkv')
    else:
        print("'mkv' directory was not found!")
        print(mkv_dir)
        sys.exit()
        
    if not mkv_paths:
        print("MKV-file(s) not found in:")
        print(mkv_dir)
        sys.exit()
    
    for i, mkv_path in enumerate(mkv_paths):
    
        print("Processing MKV-file (%d/%d)" %(i+1,len(mkv_paths)))
    
        # Speficy output directory based on MKV file name
        mkv_dir, mkv_name = os.path.split(mkv_path)
        mkv_name = os.path.splitext(mkv_name)[0]
        output_dir = os.path.join(os.path.dirname(mkv_dir), "preprocessed", mkv_name)
        
        # If output directory exists, it will not be overwritten
        if os.path.isdir(output_dir):
            print("Output directory: %s" %output_dir)
            print("already exists, skipping...")
            continue
        
        # Need to provide absolute paths for process_mkv.exe
        inpath_abs = os.path.abspath(mkv_path)
        outpath_abs = os.path.abspath(output_dir)
        exepath_abs = os.path.abspath(PROCESS_MKV_PATH)

        # Sequence index is provided 
        subprocess.run([exepath_abs, "--input=%s"%inpath_abs, 
            "--output=%s"%outpath_abs, "--sequence_idx=%d"%(i+1)], shell=True)

    createRtabmapConfig(dataset_dir)
    