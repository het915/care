#!/usr/bin/env python3
"""
Test script to verify Gemini API connection and basic functionality
"""

import requests
import base64
import json
import cv2
import numpy as np
import sys


def test_gemini_api(api_key):
    """Test Gemini API with a simple image"""
    
    print("Testing Gemini Vision API...")
    
    # Create a simple test image
    image = np.ones((480, 640, 3), dtype=np.uint8) * 255
    cv2.putText(image, "TEST IMAGE", (200, 240), 
                cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 3)
    
    # Encode to base64
    _, buffer = cv2.imencode('.jpg', image)
    image_base64 = base64.b64encode(buffer).decode('utf-8')
    
    # Prepare API request
    url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent?key={api_key}"
    
    payload = {
        "contents": [{
            "parts": [
                {"text": "What do you see in this image? Respond in one sentence."},
                {
                    "inline_data": {
                        "mime_type": "image/jpeg",
                        "data": image_base64
                    }
                }
            ]
        }],
        "generationConfig": {
            "temperature": 0.1,
            "maxOutputTokens": 100
        }
    }
    
    try:
        response = requests.post(url, json=payload)
        
        if response.status_code == 200:
            result = response.json()
            if 'candidates' in result:
                text = result['candidates'][0]['content']['parts'][0]['text']
                print(f"✓ API Test Successful!")
                print(f"  Response: {text}")
                return True
        else:
            print(f"✗ API Test Failed: Status {response.status_code}")
            print(f"  Error: {response.text}")
            return False
            
    except Exception as e:
        print(f"✗ API Test Failed: {e}")
        return False


def test_booster_sdk():
    """Test if Booster SDK is installed"""
    print("\nTesting Booster SDK installation...")
    
    try:
        from booster_robotics_sdk_python import B1LocoClient
        print("✓ Booster SDK found!")
        return True
    except ImportError as e:
        print("✗ Booster SDK not found")
        if "GLIBCXX" in str(e):
            print("  Issue: Library compatibility problem with conda/anaconda")
            print("  Solution: Use system Python instead: /usr/bin/python3")
            print("  The SDK is installed but requires system libraries")
        else:
            print("  To install: cd booster_robotics_sdk/build")
            print("  cmake .. -DBUILD_PYTHON_BINDING=on && make && sudo make install")
        return False


def main():
    """Run all tests"""
    print("=" * 60)
    print("K1 Gemini Follower - System Test")
    print("=" * 60)
    
    # Test API key
    api_key = "AIzaSyDnpcOTRkPfB0zagID3uhieCUTcpBG8jmg"
    
    # Run tests
    api_ok = test_gemini_api(api_key)
    sdk_ok = test_booster_sdk()
    
    print("\n" + "=" * 60)
    print("Test Results:")
    print(f"  Gemini API: {'✓ PASS' if api_ok else '✗ FAIL'}")
    print(f"  Booster SDK: {'✓ PASS' if sdk_ok else '✗ FAIL (optional)'}")
    
    if api_ok:
        print("\nSystem ready for person following!")
    else:
        print("\nPlease fix the issues above before running the follower.")
        
    print("=" * 60)
    
    return 0 if api_ok else 1


if __name__ == "__main__":
    sys.exit(main())