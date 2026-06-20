import urllib.request
import json

try:
    print("Fetching Anaconda package metadata...")
    url = "https://api.anaconda.org/package/conda-forge/pybullet"
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    with urllib.request.urlopen(req) as response:
        data = json.loads(response.read().decode('utf-8'))
    
    print("Parsing files...")
    files = data.get('files', [])
    win_files = []
    for f in files:
        # We look for win-64 packages
        if f.get('attrs', {}).get('platform') == 'win-64':
            filename = f.get('basename', '')
            version = f.get('version', '')
            py_ver = f.get('attrs', {}).get('python', '')
            download_url = "https:" + f.get('download_url', '')
            win_files.append({
                'filename': filename,
                'version': version,
                'py_ver': py_ver,
                'download_url': download_url
            })
            
    print(f"Found {len(win_files)} Windows 64-bit files:")
    for wf in win_files[-15:]: # print recent 15
        print(f"Ver: {wf['version']}, Python: {wf['py_ver']}, File: {wf['filename']}")
        print(f"  URL: {wf['download_url']}")
        
except Exception as e:
    print("Error:", e)
