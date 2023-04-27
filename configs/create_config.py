import os
import glob
import pathlib

def createRtabmapConfig(dataset_dir):

    # Create the output folder
    #parent_folder = os.path.abspath(os.path.join(data_path, '..', '..'))
    #rtabmap_subfolder = os.path.join(parent_folder, 'rtabmap', os.path.basename(data_path))
    #rtabmap_subfolder = os.path.join(dataset_dir, 'rtabmap', os.path.basename(data_path))
    #os.makedirs(rtabmap_subfolder, exist_ok=True)  
    rtabmap_folder = os.path.join(dataset_dir, "rtabmap/")
    rtabmap_subfolders = []
    
    data_paths = glob.glob(dataset_dir + '/preprocessed/*')
    
    ''' Create single-session config(s) '''
    
    for data_path in data_paths:
    
        sequence = os.path.basename(data_path)
        rtabmap_subfolder = os.path.join(rtabmap_folder, sequence)
        os.makedirs(rtabmap_subfolder, exist_ok=True)
        rtabmap_subfolders.append(rtabmap_subfolder)

        # Read the single-session.ini file
        script_path = os.path.dirname(os.path.realpath(__file__))
        ini_file = os.path.join(script_path, 'single-session.ini')
        with open(ini_file, 'r') as file:
            lines = file.readlines()

        # Modify the lines containing calibrationName, path_rgb, path_depth, workingDirectory
        modified_lines = []
        for line in lines:
            if line.startswith('calibrationName ='):
                abspath = os.path.abspath(os.path.join(data_path, "depth_cam", "calibration_1024x1024.yaml"))
                abspath = abspath.replace("\\","/")
                line = f'calibrationName = "{abspath}"\n'
            elif line.startswith('RGBDImages\\path_rgb ='):
                abspath = os.path.abspath(os.path.join(data_path, "depth_cam", "rgb"))
                abspath = abspath.replace("\\","/")
                line = f'RGBDImages\\path_rgb = "{abspath}"\n'
            elif line.startswith('RGBDImages\\path_depth ='):
                abspath = os.path.abspath(os.path.join(data_path, "depth_cam", "depth"))
                abspath = abspath.replace("\\","/")
                line = f'RGBDImages\\path_depth = "{abspath}"\n'
            elif line.startswith('Rtabmap\\WorkingDirectory ='):
                abspath = os.path.abspath(rtabmap_folder)
                abspath = abspath.replace("\\","/")
                line = f'Rtabmap\\WorkingDirectory = "{abspath}"\n'
            modified_lines.append(line)

        # Write the modified lines to a new file
        output_file = os.path.join(rtabmap_subfolder, 'single-session.ini')
        with open(output_file, 'w') as file:
            file.writelines(modified_lines)

    ''' Create multi-session config '''
    
    # Read the multi-session.ini file
    script_path = os.path.dirname(os.path.realpath(__file__))
    ini_file = os.path.join(script_path, 'multi-session.ini')
    with open(ini_file, 'r') as file:
        lines = file.readlines()
    
    # Modify the lines containing Database\path
    modified_lines = []
    for line in lines:
        if line.startswith('Database\path ='):
            abspath = ""
            for subfolder in rtabmap_subfolders:
                abspath += os.path.abspath(os.path.join(subfolder,"map.db;"))
            abspath = abspath.replace("\\","/")
            line = f'Database\\path = "{abspath}"\n'
        elif line.startswith('Rtabmap\\WorkingDirectory ='):
            abspath = os.path.abspath(rtabmap_folder)
            abspath = abspath.replace("\\","/")
            line = f'Rtabmap\\WorkingDirectory = "{abspath}"\n'
        modified_lines.append(line)
    
    # Write the modified lines to a new file
    output_file = os.path.join(rtabmap_folder, 'multi-session.ini')
    with open(output_file, 'w') as file:
        file.writelines(modified_lines)