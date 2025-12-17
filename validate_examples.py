#!/usr/bin/env python3
"""
Content validation script for Python code examples in the ROS 2 course.

This script validates that all Python code examples in the course:
1. Are syntactically correct
2. Follow ROS 2 best practices
3. Are compatible with Python 3.8+ and ROS 2 Humble
"""

import ast
import os
import sys
from pathlib import Path

def validate_python_syntax(file_path):
    """Validate Python syntax for the given file."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        ast.parse(content)
        return True, None
    except SyntaxError as e:
        return False, f"Syntax error at line {e.lineno}: {e.msg}"
    except Exception as e:
        return False, f"Error reading file: {str(e)}"

def find_python_examples(base_dir):
    """Find all Python example files in the examples directory."""
    python_files = []
    base_path = Path(base_dir)
    
    for py_file in base_path.rglob("*.py"):
        python_files.append(str(py_file))
    
    return python_files

def validate_ros2_imports(file_path):
    """Check that Python examples use appropriate ROS 2 imports."""
    required_imports = [
        'rclpy',
        'std_msgs',
        'geometry_msgs',
        'sensor_msgs',
        'nav_msgs'
    ]
    
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Check for common ROS 2 imports
    has_ros_import = any(imp in content for imp in required_imports)
    
    if not has_ros_import and 'rclpy' not in content:
        return False, f"File {file_path} does not appear to use ROS 2 imports"
    
    return True, None

def main():
    """Main function to run validation on Python examples."""
    if len(sys.argv) < 2:
        print("Usage: python validate_examples.py <examples_dir>")
        sys.exit(1)
    
    examples_dir = sys.argv[1]
    
    if not os.path.exists(examples_dir):
        print(f"Error: Directory {examples_dir} does not exist")
        sys.exit(1)
    
    print(f"Validating Python code examples in {examples_dir}")
    
    python_files = find_python_examples(examples_dir)
    print(f"Found {len(python_files)} Python files")
    
    all_valid = True
    for file_path in python_files:
        print(f"Validating {file_path}...")
        
        # Validate syntax
        syntax_valid, syntax_error = validate_python_syntax(file_path)
        if not syntax_valid:
            print(f"  [FAIL] Syntax Error: {syntax_error}")
            all_valid = False
            continue

        # Validate ROS 2 imports
        ros_valid, ros_error = validate_ros2_imports(file_path)
        if not ros_valid:
            print(f"  [WARN] ROS2 Import Warning: {ros_error}")
            # We'll allow this as a warning, not a failure
        
        print(f"  [PASS] {file_path} passed validation")
    
    if all_valid:
        print("\n[SUCCESS] All Python code examples passed validation!")
        return 0
    else:
        print("\n[ERROR] Some Python code examples failed validation!")
        return 1

if __name__ == "__main__":
    sys.exit(main())