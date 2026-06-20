import urllib.request
import re

try:
    print("Fetching PyPI simple index...")
    req = urllib.request.Request(
        'https://pypi.org/simple/',
        headers={'User-Agent': 'Mozilla/5.0'}
    )
    with urllib.request.urlopen(req) as response:
        html = response.read().decode('utf-8')
    
    print("Searching for packages...")
    packages = []
    matches = re.findall(r'<a href="/simple/([^/]+)/"', html)
    for name in matches:
        if 'pybullet' in name.lower():
            packages.append(name)
            
    print("Found packages:")
    print(packages)
except Exception as e:
    print("Error:", e)
