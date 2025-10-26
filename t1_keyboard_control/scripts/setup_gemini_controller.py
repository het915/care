#!/usr/bin/env python3
"""
Gemini Robot Controller Setup and Test Script
Helps configure environment and test the AI robot controller
"""

import os
import sys
import subprocess
import requests
import time

def check_environment():
    """Check if all required dependencies are available"""
    print("🔍 Checking environment...")
    
    # Check Python packages
    required_packages = [
        'google.generativeai',
        'cv2',
        'PIL',
        'numpy',
        'requests'
    ]
    
    missing_packages = []
    for package in required_packages:
        try:
            __import__(package)
            print(f"✅ {package} - OK")
        except ImportError:
            missing_packages.append(package)
            print(f"❌ {package} - Missing")
    
    if missing_packages:
        print(f"\n📦 Installing missing packages: {', '.join(missing_packages)}")
        return False
    
    return True

def check_gemini_api():
    """Check if Gemini API key is configured"""
    print("\n🔑 Checking Gemini API configuration...")
    
    api_key = os.getenv('GEMINI_API_KEY')
    if not api_key:
        print("❌ GEMINI_API_KEY environment variable not set")
        print("\n📋 To set up Gemini API:")
        print("1. Get API key from https://aistudio.google.com/app/apikey")
        print("2. export GEMINI_API_KEY='your_api_key_here'")
        print("3. Add to ~/.bashrc for permanent setup")
        return False
    
    print(f"✅ GEMINI_API_KEY configured (length: {len(api_key)})")
    
    # Test API connection
    try:
        import google.generativeai as genai
        genai.configure(api_key=api_key)
        
        model = genai.GenerativeModel('gemini-1.5-flash')
        response = model.generate_content("Hello, test connection")
        print("✅ Gemini API connection successful")
        return True
    except Exception as e:
        print(f"❌ Gemini API test failed: {e}")
        return False

def check_k1_robot():
    """Check K1 robot connectivity"""
    print("\n🤖 Checking K1 robot connectivity...")
    
    k1_addresses = [
        "192.168.10.102",
        "192.168.0.102", 
        "192.168.1.102"
    ]
    
    for addr in k1_addresses:
        try:
            response = requests.get(f"http://{addr}:8080", timeout=2)
            if response.status_code == 200:
                print(f"✅ K1 robot found at {addr}")
                return addr
        except:
            print(f"⏳ Checking {addr}...")
            continue
    
    print("❌ K1 robot not found on common addresses")
    print("📋 Make sure:")
    print("- K1 robot is powered on")
    print("- Connected to same network")
    print("- Camera service is running")
    return None

def build_workspace():
    """Build ROS2 workspace"""
    print("\n🔨 Building ROS2 workspace...")
    
    try:
        # Change to workspace directory
        workspace_dir = "/home/het/ros2_ws"
        os.chdir(workspace_dir)
        
        # Build the package
        result = subprocess.run(
            ["colcon", "build", "--packages-select", "t1_keyboard_control"],
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            print("✅ Workspace build successful")
            print("📋 Remember to source: source install/setup.bash")
            return True
        else:
            print(f"❌ Build failed: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"❌ Build error: {e}")
        return False

def test_launch():
    """Test launch file"""
    print("\n🚀 Testing launch configuration...")
    
    launch_file = "/home/het/ros2_ws/src/t1_keyboard_control/launch/gemini_robot_controller_launch.py"
    
    if os.path.exists(launch_file):
        print("✅ Launch file exists")
        print(f"📁 Path: {launch_file}")
        
        print("\n📋 To launch the system:")
        print("1. source /home/het/ros2_ws/install/setup.bash")
        print("2. export GEMINI_API_KEY='your_api_key'")
        print("3. ros2 launch t1_keyboard_control gemini_robot_controller_launch.py")
        
        return True
    else:
        print("❌ Launch file not found")
        return False

def create_test_commands():
    """Create test commands file"""
    print("\n📝 Creating test commands...")
    
    test_commands = """#!/bin/bash
# Gemini Robot Controller Test Commands

# 1. Source ROS2 environment
source /opt/ros/humble/setup.bash
source /home/het/ros2_ws/install/setup.bash

# 2. Set Gemini API key (replace with your key)
export GEMINI_API_KEY="your_gemini_api_key_here"

# 3. Launch the complete system
echo "🚀 Launching Gemini Robot Controller..."
ros2 launch t1_keyboard_control gemini_robot_controller_launch.py

# Alternative: Launch individual nodes for debugging
# ros2 run t1_keyboard_control k1_camera_agent
# ros2 run t1_keyboard_control gemini_robot_controller
# ros2 run t1_keyboard_control image_converter

# Test voice commands (in separate terminal):
# ros2 topic pub /voice_command std_msgs/String "data: 'find the red ball'"
# ros2 topic pub /voice_command std_msgs/String "data: 'look for a person'"
# ros2 topic pub /voice_command std_msgs/String "data: 'approach the object'"
"""
    
    script_path = "/home/het/ros2_ws/src/t1_keyboard_control/scripts/test_gemini_controller.sh"
    with open(script_path, 'w') as f:
        f.write(test_commands)
    
    os.chmod(script_path, 0o755)
    print(f"✅ Test script created: {script_path}")

def main():
    """Main setup and test function"""
    print("🎯 Gemini Robot Controller Setup & Test")
    print("=" * 50)
    
    # Run all checks
    env_ok = check_environment()
    api_ok = check_gemini_api()
    k1_addr = check_k1_robot()
    
    # Build workspace
    build_ok = build_workspace()
    
    # Test launch setup
    launch_ok = test_launch()
    
    # Create test commands
    create_test_commands()
    
    print("\n" + "=" * 50)
    print("📊 SETUP SUMMARY")
    print("=" * 50)
    print(f"Environment:     {'✅' if env_ok else '❌'}")
    print(f"Gemini API:      {'✅' if api_ok else '❌'}")
    print(f"K1 Robot:        {'✅' if k1_addr else '❌'}")
    print(f"Build:           {'✅' if build_ok else '❌'}")
    print(f"Launch Config:   {'✅' if launch_ok else '❌'}")
    
    if all([env_ok, api_ok, build_ok, launch_ok]):
        print("\n🎉 System ready for testing!")
        print("\n📋 Next steps:")
        print("1. Set your GEMINI_API_KEY")
        print("2. Run: ./scripts/test_gemini_controller.sh")
        print("3. Test voice commands like 'find the red ball'")
        
        if k1_addr:
            print(f"4. K1 robot available at {k1_addr}")
        
    else:
        print("\n⚠️  Some issues need to be resolved before testing")

if __name__ == "__main__":
    main()