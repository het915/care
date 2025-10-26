#!/usr/bin/env python3
"""
K1 Robot Diagnostic Tool
Quick tool to diagnose K1 robot services and capabilities
"""

import requests
import socket
import cv2
from urllib.parse import urljoin

def test_robot_connection(robot_ip):
    """Test basic connection to K1 robot"""
    print(f"🔍 Testing K1 Robot at {robot_ip}")
    
    # Test basic connectivity
    try:
        result = os.system(f"ping -c 1 -W 2 {robot_ip} > /dev/null 2>&1")
        if result == 0:
            print(f"✅ Robot is reachable at {robot_ip}")
        else:
            print(f"❌ Robot not reachable at {robot_ip}")
            return False
    except:
        print(f"❌ Cannot test connectivity to {robot_ip}")
        return False

    # Test common robot ports
    common_ports = [22, 80, 443, 8080, 8000, 5000, 9090, 8081, 8888, 1935]
    open_ports = []
    
    for port in common_ports:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        try:
            result = sock.connect_ex((robot_ip, port))
            if result == 0:
                open_ports.append(port)
                print(f"✅ Port {port} is open")
            else:
                print(f"❌ Port {port} is closed")
        except:
            print(f"❌ Cannot test port {port}")
        finally:
            sock.close()
    
    # Test common camera URLs
    camera_urls = [
        f"http://{robot_ip}:8080/video",
        f"http://{robot_ip}:8080/stream", 
        f"http://{robot_ip}:8080/camera",
        f"http://{robot_ip}:8080/mjpeg",
        f"http://{robot_ip}:8080/video.mjpg",
        f"http://{robot_ip}:8000/video",
        f"http://{robot_ip}/video",
        f"http://{robot_ip}/camera",
        f"http://{robot_ip}/stream",
        f"rtsp://{robot_ip}:8554/video",
        f"rtsp://{robot_ip}/video",
    ]
    
    working_urls = []
    for url in camera_urls:
        try:
            print(f"🔍 Testing: {url}")
            response = requests.get(url, timeout=5, stream=True)
            if response.status_code == 200:
                print(f"✅ Camera URL working: {url}")
                working_urls.append(url)
                # Try to test if it's actually video
                if 'video' in response.headers.get('content-type', '').lower() or \
                   'image' in response.headers.get('content-type', '').lower():
                    print(f"📹 Confirmed video stream: {url}")
            else:
                print(f"❌ URL returned {response.status_code}: {url}")
        except requests.exceptions.RequestException as e:
            print(f"❌ URL failed: {url} - {e}")
    
    # Test with OpenCV
    print("\n🎬 Testing video capture with OpenCV...")
    for url in working_urls[:3]:  # Test top 3 working URLs
        try:
            print(f"🔍 Testing OpenCV capture: {url}")
            cap = cv2.VideoCapture(url)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    print(f"✅ OpenCV can capture from: {url}")
                    print(f"📐 Frame size: {frame.shape}")
                else:
                    print(f"❌ OpenCV cannot read frames from: {url}")
            else:
                print(f"❌ OpenCV cannot open: {url}")
            cap.release()
        except Exception as e:
            print(f"❌ OpenCV error with {url}: {e}")
    
    return working_urls, open_ports

def test_web_interface(robot_ip):
    """Test if robot has a web interface"""
    web_urls = [
        f"http://{robot_ip}",
        f"http://{robot_ip}:8080",
        f"http://{robot_ip}:8000",
        f"https://{robot_ip}",
    ]
    
    print("\n🌐 Testing web interfaces...")
    for url in web_urls:
        try:
            response = requests.get(url, timeout=5)
            if response.status_code == 200:
                print(f"✅ Web interface found: {url}")
                print(f"📄 Content type: {response.headers.get('content-type', 'unknown')}")
                if len(response.text) < 1000:
                    print(f"📝 Content preview: {response.text[:200]}...")
            else:
                print(f"❌ Web interface returned {response.status_code}: {url}")
        except Exception as e:
            print(f"❌ Web interface failed: {url} - {e}")

if __name__ == "__main__":
    import os
    robot_ip = "192.168.10.102"  # Found from network scan
    
    print("🤖 K1 Robot Diagnostic Tool")
    print("=" * 50)
    
    working_urls, open_ports = test_robot_connection(robot_ip)
    test_web_interface(robot_ip)
    
    print("\n📊 Summary:")
    print(f"🔗 Open ports: {open_ports}")
    print(f"📹 Working camera URLs: {working_urls}")
    
    if working_urls:
        print(f"\n🎯 Recommended camera URL: {working_urls[0]}")
    else:
        print("\n❌ No working camera URLs found")
        print("💡 Try checking:")
        print("   - Robot camera service is running")
        print("   - Robot is fully booted")
        print("   - Network configuration")