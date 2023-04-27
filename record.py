import os
import sys
import subprocess

if __name__ == '__main__':

    K4A_RECORDER_PATH = "C:/Program Files/Azure Kinect SDK v1.4.1/tools/k4arecorder.exe"

    if len(sys.argv) != 2:
        print("Invalid number of input argumens!")
        print("Usage: python record.py output.mkv")
        sys.exit()
        
    # Check that k4a_recorder.exe is found
    if not os.path.exists(K4A_RECORDER_PATH):
        print("k4a_recorder.exe not found!")
        print("Update K4A_RECORDER_PATH that is currently")
        print(K4A_RECORDER_PATH)
        sys.exit()
        
    # Add file extension (.mkv) if not given
    mkv_name = sys.argv[1]
    if '.mkv' not in mkv_name:
        mkv_name = mkv_name + '.mkv'
        
    cwd = os.getcwd()
    outpath = os.path.join(cwd, mkv_name)
        
    try:
        subprocess.run([K4A_RECORDER_PATH, "--color-mode", "720p", "--depth-mode", "WFOV_2X2BINNED", 
        "--rate", "30", outpath], shell=True)
    except KeyboardInterrupt:
        print("Output written to:")
        print(outpath)
    