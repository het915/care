#!/usr/bin/env python3
"""
K1 Robot Camera Agent
Intelligent agent that automatically discovers, connects to, and streams K1 robot camera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import requests
import socket
import subprocess
import time
import json
import yaml
from datetime import datetime
import ipaddress
import os
import netifaces

class K1CameraAgent(Node):
    def __init__(self):
        super().__init__('k1_camera_agent')
        
        # Agent configuration
        self.robot_ip = None
        self.camera_url = None
        self.connection_established = False
        self.discovery_complete = False
        
        # Camera streaming
        self.bridge = CvBridge()
        self.cap = None
        self.running = False
        self.frame_count = 0
        
        # Publishers
        self.image_publisher = self.create_publisher(Image, '/k1_camera/image_raw', 10)
        
        # Status tracking
        self.discovery_attempts = 0
        self.max_discovery_attempts = 3
        self.last_frame_time = time.time()
        
        self.get_logger().info("🤖 K1 Camera Agent Starting...")
        self.get_logger().info("🔍 Beginning intelligent robot discovery...")
        
        # Start the intelligent discovery process
        self.discovery_thread = threading.Thread(target=self.intelligent_discovery, daemon=True)
        self.discovery_thread.start()
        
        # Status timer
        self.create_timer(10.0, self.report_status)
    
    def intelligent_discovery(self):
        """Intelligent discovery process for K1 robot"""
        self.get_logger().info("🧠 Starting intelligent K1 robot discovery...")
        
        # Step 1: Network Analysis
        network_info = self.analyze_network_topology()
        
        # Step 2: Robot Discovery
        robot_candidates = self.discover_robot_candidates(network_info)
        
        # Step 3: Camera Service Detection
        camera_info = self.detect_camera_services(robot_candidates)
        
        # Step 4: Connection Establishment
        success = self.establish_camera_connection(camera_info)
        
        if success:
            self.start_streaming()
        else:
            self.get_logger().error("❌ Failed to establish camera connection")
            self.fallback_discovery()
    
    def analyze_network_topology(self):
        """Analyze network topology to find potential robots."""
        interfaces = netifaces.interfaces()
        ethernet_interfaces = []
        
        for iface in interfaces:
            if iface.startswith(('eth', 'enx', 'enp')):
                addrs = netifaces.ifaddresses(iface)
                if netifaces.AF_INET in addrs:
                    for addr_info in addrs[netifaces.AF_INET]:
                        ip = addr_info['addr']
                        netmask = addr_info.get('netmask', '255.255.255.0')
                        ethernet_interfaces.append((iface, ip, netmask))
                        self.get_logger().info(f"🔗 Found Ethernet interface: {iface} ({ip})")
        
        robot_candidates = set()
        
        # Start with the discovered K1 robot IP
        k1_robot_ip = '192.168.10.102'
        if self.ping_host(k1_robot_ip):
            robot_candidates.add(k1_robot_ip)
            self.get_logger().info(f"🎯 Found K1 robot: {k1_robot_ip}")
        
        # Common robot IP addresses to test as fallback
        common_robot_ips = [
            '192.168.1.100', '192.168.1.10', '192.168.1.1', '192.168.1.102',
            '192.168.0.100', '192.168.0.10', '192.168.0.1', '192.168.0.102', 
            '192.168.10.100', '192.168.10.1', '192.168.10.101', '192.168.10.103',
            '10.0.0.100', '10.0.0.10', '10.0.0.1', '10.0.0.102',
            '172.16.0.100', '172.16.0.10', '172.16.0.1', '172.16.0.102'
        ]
        
        for ip in common_robot_ips:
            if ip not in robot_candidates and self.ping_host(ip):
                robot_candidates.add(ip)
                self.get_logger().info(f"🎯 Found robot candidate: {ip}")
    
    def discover_robot_candidates(self, network_info):
        """Discover potential K1 robot candidates"""
        self.get_logger().info("🔍 Scanning for K1 robot candidates...")
        
        robot_candidates = []
        
        # Common K1 robot IP patterns
        common_robot_ips = [
            '192.168.1.100', '192.168.1.10', '192.168.1.1',
            '192.168.0.100', '192.168.0.10', '192.168.0.1',
            '192.168.10.100', '192.168.10.10', '192.168.10.1',
            '10.0.0.100', '10.0.0.10', '10.0.0.1',
            '172.16.0.100', '172.16.0.10', '172.16.0.1'
        ]
        
        # Test common IPs first
        for ip in common_robot_ips:
            if self.ping_host(ip):
                robot_info = self.analyze_robot_candidate(ip)
                if robot_info:
                    robot_candidates.append(robot_info)
                    self.get_logger().info(f"🎯 Found robot candidate: {ip}")
        
        # Network scan on detected subnets
        for subnet in network_info['subnets']:
            try:
                network = ipaddress.IPv4Network(subnet, strict=False)
                self.get_logger().info(f"🌐 Scanning subnet: {subnet}")
                
                # Scan common robot IPs in subnet
                for host_num in [1, 10, 100, 110, 200]:
                    try:
                        ip = str(network.network_address + host_num)
                        if ip not in [c['ip'] for c in robot_candidates]:
                            if self.ping_host(ip, timeout=1):
                                robot_info = self.analyze_robot_candidate(ip)
                                if robot_info:
                                    robot_candidates.append(robot_info)
                                    self.get_logger().info(f"🎯 Found robot candidate: {ip}")
                    except:
                        continue
                        
            except Exception as e:
                self.get_logger().warn(f"Error scanning subnet {subnet}: {e}")
        
        self.get_logger().info(f"🔍 Found {len(robot_candidates)} robot candidates")
        return robot_candidates
    
    def ping_host(self, ip, timeout=2):
        """Ping a host to check connectivity"""
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', str(timeout), ip],
                capture_output=True,
                text=True,
                timeout=timeout + 1
            )
            return result.returncode == 0
        except:
            return False
    
    def analyze_robot_candidate(self, ip):
        """Analyze a potential robot to determine if it's a K1"""
        try:
            robot_info = {
                'ip': ip,
                'services': [],
                'robot_type': 'unknown',
                'camera_services': [],
                'confidence_score': 0
            }
            
            # Test common robot service ports
            service_ports = [8080, 8000, 5000, 80, 443, 9090, 11311]
            
            for port in service_ports:
                if self.test_tcp_port(ip, port):
                    service_info = self.probe_service(ip, port)
                    if service_info:
                        robot_info['services'].append(service_info)
                        
                        # Check if it's a K1 robot
                        if self.is_k1_robot_service(service_info):
                            robot_info['robot_type'] = 'k1'
                            robot_info['confidence_score'] += 30
            
            # Return only if we have some confidence it's a robot
            return robot_info if robot_info['confidence_score'] > 0 else None
            
        except Exception as e:
            self.get_logger().debug(f"Error analyzing {ip}: {e}")
            return None
    
    def test_tcp_port(self, ip, port, timeout=2):
        """Test if a TCP port is open"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            result = sock.connect_ex((ip, port))
            sock.close()
            return result == 0
        except:
            return False
    
    def probe_service(self, ip, port):
        """Probe a service to get information"""
        service_info = {
            'ip': ip,
            'port': port,
            'type': 'unknown',
            'response': None
        }
        
        try:
            # HTTP probe
            for protocol in ['http', 'https']:
                try:
                    url = f"{protocol}://{ip}:{port}"
                    response = requests.get(url, timeout=3)
                    service_info['type'] = 'http'
                    service_info['response'] = response.text[:200]  # First 200 chars
                    service_info['status_code'] = response.status_code
                    break
                except:
                    continue
                    
        except Exception as e:
            self.get_logger().debug(f"Error probing {ip}:{port}: {e}")
        
        return service_info
    
    def is_k1_robot_service(self, service_info):
        """Check if service belongs to K1 robot"""
        if not service_info.get('response'):
            return False
        
        response = service_info['response'].lower()
        k1_indicators = [
            'k1', 'robot', 'camera', 'video', 'stream',
            'ros', 'robotic', 'unitree', 'quadruped'
        ]
        
        return any(indicator in response for indicator in k1_indicators)
    
    def detect_camera_services(self, robot_candidates):
        """Detect camera services on robot candidates"""
        self.get_logger().info("📹 Detecting camera services...")
        
        camera_services = []
        
        for robot in robot_candidates:
            self.get_logger().info(f"🔍 Checking camera services on {robot['ip']}")
            
            # Common camera service paths
            camera_paths = [
                '/video', '/camera', '/stream', '/video_feed',
                '/mjpeg', '/webcam', '/cam', '/live'
            ]
            
            # Common camera ports
            camera_ports = [8080, 8000, 5000, 80]
            
            for service in robot['services']:
                if service['type'] == 'http':
                    for path in camera_paths:
                        camera_url = f"http://{service['ip']}:{service['port']}{path}"
                        if self.test_camera_stream(camera_url):
                            camera_info = {
                                'robot_ip': robot['ip'],
                                'camera_url': camera_url,
                                'port': service['port'],
                                'path': path,
                                'confidence': robot['confidence_score'] + 20
                            }
                            camera_services.append(camera_info)
                            self.get_logger().info(f"📹 Found camera service: {camera_url}")
        
        # Sort by confidence score
        camera_services.sort(key=lambda x: x['confidence'], reverse=True)
        
        return camera_services
    
    def test_camera_stream(self, url):
        """Test if URL provides a valid camera stream"""
        try:
            cap = cv2.VideoCapture(url)
            if cap.isOpened():
                ret, frame = cap.read()
                cap.release()
                return ret and frame is not None
            return False
        except:
            return False
    
    def establish_camera_connection(self, camera_services):
        """Establish connection to the best camera service"""
        if not camera_services:
            self.get_logger().error("❌ No camera services found")
            return False
        
        self.get_logger().info(f"🔗 Attempting to connect to {len(camera_services)} camera services...")
        
        for camera_info in camera_services:
            try:
                self.get_logger().info(f"🎯 Connecting to: {camera_info['camera_url']}")
                
                # Handle USB camera
                if camera_info['camera_url'].startswith("usb:"):
                    device_id = int(camera_info['camera_url'].split(":")[1])
                    cap = cv2.VideoCapture(device_id)
                    if cap.isOpened():
                        # Set USB camera properties
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        cap.set(cv2.CAP_PROP_FPS, 30)
                else:
                    # Handle network camera
                    cap = cv2.VideoCapture(camera_info['camera_url'])
                
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        self.robot_ip = camera_info['robot_ip']
                        self.camera_url = camera_info['camera_url']
                        self.cap = cap
                        self.connection_established = True
                        
                        camera_type = "USB camera" if self.camera_url.startswith("usb:") else "network camera"
                        self.get_logger().info(f"✅ Successfully connected to {camera_type}!")
                        self.get_logger().info(f"📡 Source: {self.robot_ip}")
                        self.get_logger().info(f"📹 Camera: {self.camera_url}")
                        
                        # Save configuration
                        self.save_robot_config(camera_info)
                        
                        return True
                    else:
                        cap.release()
                        
            except Exception as e:
                self.get_logger().debug(f"Connection failed for {camera_info['camera_url']}: {e}")
                continue
        
        return False
    
    def save_robot_config(self, camera_info):
        """Save successful robot configuration"""
        try:
            config = {
                'robot_ip': self.robot_ip,
                'camera_url': self.camera_url,
                'discovered_at': datetime.now().isoformat(),
                'camera_info': camera_info
            }
            
            config_path = os.path.expanduser('~/.ros/k1_robot_config.yaml')
            os.makedirs(os.path.dirname(config_path), exist_ok=True)
            
            with open(config_path, 'w') as f:
                yaml.dump(config, f)
            
            self.get_logger().info(f"💾 Saved robot configuration to {config_path}")
            
        except Exception as e:
            self.get_logger().warn(f"Could not save config: {e}")
    
    def load_robot_config(self):
        """Load previously saved robot configuration"""
        try:
            config_path = os.path.expanduser('~/.ros/k1_robot_config.yaml')
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                
                # Test if saved config still works
                if self.test_camera_stream(config['camera_url']):
                    self.robot_ip = config['robot_ip']
                    self.camera_url = config['camera_url']
                    
                    self.get_logger().info(f"📁 Loaded saved robot configuration")
                    self.get_logger().info(f"📡 Robot IP: {self.robot_ip}")
                    
                    return True
                else:
                    self.get_logger().info("📁 Saved config no longer valid, starting fresh discovery")
                    
        except Exception as e:
            self.get_logger().debug(f"Could not load saved config: {e}")
        
        return False
    
    def fallback_discovery(self):
        """Fallback discovery methods"""
        self.get_logger().info("🔄 Starting fallback discovery...")
        
        # Try to load saved configuration first
        if self.load_robot_config():
            self.cap = cv2.VideoCapture(self.camera_url)
            if self.cap.isOpened():
                self.connection_established = True
                self.start_streaming()
                return
        
        # Manual network scan as fallback
        self.get_logger().info("🌐 Performing comprehensive network scan...")
        
        # Get all possible IP ranges from interfaces
        try:
            for interface in netifaces.interfaces():
                if interface.startswith(('eth', 'en')):
                    addrs = netifaces.ifaddresses(interface)
                    if netifaces.AF_INET in addrs:
                        for addr in addrs[netifaces.AF_INET]:
                            ip = addr['addr']
                            network = ipaddress.IPv4Network(f"{ip}/24", strict=False)
                            
                            # Quick scan of this network
                            self.quick_network_scan(str(network))
                            
        except Exception as e:
            self.get_logger().error(f"Fallback discovery failed: {e}")
    
    def quick_network_scan(self, subnet):
        """Quick scan of a network subnet"""
        try:
            network = ipaddress.IPv4Network(subnet, strict=False)
            
            # Test common robot IPs in this subnet
            for host_num in [1, 10, 50, 100, 110, 150, 200, 250]:
                try:
                    ip = str(network.network_address + host_num)
                    if self.ping_host(ip, timeout=1):
                        # Test common camera URLs
                        for port in [8080, 8000, 5000]:
                            for path in ['/video', '/camera', '/stream']:
                                url = f"http://{ip}:{port}{path}"
                                if self.test_camera_stream(url):
                                    self.robot_ip = ip
                                    self.camera_url = url
                                    self.cap = cv2.VideoCapture(url)
                                    if self.cap.isOpened():
                                        self.connection_established = True
                                        self.get_logger().info(f"✅ Fallback discovery successful: {url}")
                                        self.start_streaming()
                                        return
                except:
                    continue
                    
        except Exception as e:
            self.get_logger().debug(f"Quick scan error: {e}")
    
    def start_streaming(self):
        """Start camera streaming"""
        if not self.connection_established:
            return
        
        self.get_logger().info("🎬 Starting camera streaming...")
        self.running = True
        
        # Start streaming thread
        self.streaming_thread = threading.Thread(target=self.streaming_loop, daemon=True)
        self.streaming_thread.start()
    
    def streaming_loop(self):
        """Main streaming loop"""
        while self.running and rclpy.ok():
            try:
                ret, frame = self.cap.read()
                
                if ret:
                    # Convert to ROS message
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'k1_camera'
                    
                    # Publish
                    self.image_publisher.publish(msg)
                    self.frame_count += 1
                    self.last_frame_time = time.time()
                    
                else:
                    self.get_logger().warn("📹 Failed to read frame, attempting reconnection...")
                    self.reconnect_camera()
                    
            except Exception as e:
                self.get_logger().error(f"Streaming error: {e}")
                time.sleep(1)
    
    def reconnect_camera(self):
        """Reconnect to camera"""
        try:
            if self.cap:
                self.cap.release()
                time.sleep(2)
            
            self.cap = cv2.VideoCapture(self.camera_url)
            if self.cap.isOpened():
                self.get_logger().info("🔄 Reconnected to camera")
            else:
                self.get_logger().error("❌ Failed to reconnect")
                
        except Exception as e:
            self.get_logger().error(f"Reconnection error: {e}")
    
    def report_status(self):
        """Report agent status"""
        if self.connection_established:
            time_since_frame = time.time() - self.last_frame_time
            self.get_logger().info(f"📊 Agent Status: Connected | Frames: {self.frame_count} | Robot: {self.robot_ip}")
            
            if time_since_frame > 10:
                self.get_logger().warn(f"⚠️ No frames received for {time_since_frame:.1f}s")
        else:
            self.get_logger().info("📊 Agent Status: Discovering robot...")
    
    def __del__(self):
        """Cleanup"""
        self.running = False
        if self.cap:
            self.cap.release()
    
    def setup_usb_camera_fallback(self):
        """Set up USB camera as fallback when network camera not available."""
        self.get_logger().info("📹 Setting up USB camera fallback...")
        
        # Try different USB camera indices
        for device_id in [0, 1, 2, 3]:
            try:
                test_cap = cv2.VideoCapture(device_id)
                if test_cap.isOpened():
                    ret, frame = test_cap.read()
                    if ret:
                        self.get_logger().info(f"✅ Found USB camera at /dev/video{device_id}")
                        test_cap.release()
                        # Set up for USB camera streaming
                        self.robot_ip = "USB_Camera"
                        self.camera_url = f"usb:{device_id}"
                        return "USB_Camera", f"usb:{device_id}"
                test_cap.release()
            except Exception as e:
                self.get_logger().debug(f"USB camera {device_id} failed: {e}")
        
        return None, None
    
    def connect_to_usb_camera(self, device_id=0):
        """Connect to USB camera directly."""
        try:
            self.cap = cv2.VideoCapture(device_id)
            if self.cap.isOpened():
                # Set camera properties
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                
                self.get_logger().info(f"📹 Connected to USB camera /dev/video{device_id}")
                self.connection_established = True
                return True
            else:
                self.get_logger().error(f"❌ Cannot open USB camera /dev/video{device_id}")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ USB camera connection failed: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    
    agent = None
    try:
        agent = K1CameraAgent()
        agent.get_logger().info("🚀 K1 Camera Agent initialized successfully")
        rclpy.spin(agent)
    except KeyboardInterrupt:
        if agent:
            agent.get_logger().info("🛑 Agent stopped by user")
    except Exception as e:
        if agent:
            agent.get_logger().error(f"❌ Agent error: {e}")
        else:
            print(f"❌ Failed to initialize agent: {e}")
    finally:
        if agent:
            try:
                agent.destroy_node()
            except:
                pass
        try:
            rclpy.shutdown()
        except:
            pass  # Ignore shutdown errors


if __name__ == '__main__':
    main()