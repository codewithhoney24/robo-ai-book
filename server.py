# This file helps Railway detect the project as Python-based
# The main application code is in the 'backend' directory

import os
import subprocess
import sys

def deploy():
    """Run the backend application"""
    os.chdir('backend')
    cmd = [
        sys.executable, '-m', 'uvicorn', 
        'src.api.main:app', 
        '--host', '0.0.0.0', 
        '--port', os.environ.get('PORT', '8000')
    ]
    subprocess.run(cmd)

if __name__ == '__main__':
    deploy()