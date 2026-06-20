import urllib.request
import tarfile
import os
import shutil

# Target URLs and directories
URL = "https://api.anaconda.org/download/conda-forge/pybullet/3.21/win-64/pybullet-3.21-py38h5846ac1_4.tar.bz2"
SCRATCH_DIR = r"C:\Users\user\AppData\Local\Temp\pybullet_temp"
ARCHIVE_PATH = os.path.join(SCRATCH_DIR, "pybullet.tar.bz2")
VENV_SITE_PACKAGES = r"D:\MyProj_2026\robotics-tutor-agent\.venv\Lib\site-packages"

def main():
    # 1. Ensure scratch dir exists
    if not os.path.exists(SCRATCH_DIR):
        os.makedirs(SCRATCH_DIR)
        
    # 2. Download archive
    print(f"Downloading prebuilt pybullet from {URL}...")
    req = urllib.request.Request(URL, headers={'User-Agent': 'Mozilla/5.0'})
    with urllib.request.urlopen(req) as response:
        with open(ARCHIVE_PATH, 'wb') as out_file:
            out_file.write(response.read())
    print("Download completed successfully.")
    
    # 3. Extract tar.bz2
    print("Extracting archive...")
    extract_path = os.path.join(SCRATCH_DIR, "extracted")
    if os.path.exists(extract_path):
        shutil.rmtree(extract_path)
    os.makedirs(extract_path)
    
    with tarfile.open(ARCHIVE_PATH, "r:bz2") as tar:
        tar.extractall(path=extract_path)
    print("Extraction completed.")
    
    # 4. Copy files to virtual environment
    # Inside the conda package, python files are under Lib/site-packages/
    source_site_packages = os.path.join(extract_path, "Lib", "site-packages")
    if not os.path.exists(source_site_packages):
        # some packages might put them under site-packages directly or similar
        print(f"Checking folders in extracted path: {os.listdir(extract_path)}")
        for root, dirs, files in os.walk(extract_path):
            if "site-packages" in dirs:
                source_site_packages = os.path.join(root, "site-packages")
                break
                
    print(f"Source site-packages: {source_site_packages}")
    
    if os.path.exists(source_site_packages):
        print("Copying pybullet files to virtual environment...")
        for item in os.listdir(source_site_packages):
            s_item = os.path.join(source_site_packages, item)
            d_item = os.path.join(VENV_SITE_PACKAGES, item)
            
            # Remove existing version if any
            if os.path.exists(d_item):
                if os.path.isdir(d_item):
                    shutil.rmtree(d_item)
                else:
                    os.remove(d_item)
                    
            if os.path.isdir(s_item):
                shutil.copytree(s_item, d_item)
                print(f"Copied directory: {item}")
            else:
                shutil.copy2(s_item, d_item)
                print(f"Copied file: {item}")
                
        print("Successfully installed PyBullet files!")
    else:
        print("Error: Could not find site-packages folder in extracted files.")
        
    # Cleanup temp files
    try:
        shutil.rmtree(SCRATCH_DIR)
        print("Cleaned up temporary directories.")
    except Exception as e:
        print(f"Cleanup warning: {e}")

if __name__ == "__main__":
    main()
