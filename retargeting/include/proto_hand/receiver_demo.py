import zmq
import sys
import os
import time
from typing import Optional
from numpy import rad2deg
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/include")
import sharpa_hand_pb2

class MocapKeypointsReceiver:
    """MocapKeypoints message receiver"""
    def __init__(self, address="tcp://localhost:6667"):
        self.address = address
        
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        
        self.socket.setsockopt(zmq.RCVHWM, 1)  
        self.socket.setsockopt(zmq.LINGER, 0)    
        
        self.socket.connect(self.address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        print(f"[MocapKeypoints Receiver] Connected to {address}, no topic mode")
        print("[MocapKeypoints Receiver] Waiting for messages...")

    def receive_mocap_keypoints(self) -> Optional[sharpa_hand_pb2.MocapKeypoints]:
        """
        Receive MocapKeypoints message
        """
        try:
            if self.socket.getsockopt(zmq.RCVBUF) > 8:  
                print("[MocapKeypoints Receiver] Warning: receive buffer full, dropping old messages")
                while self.socket.getsockopt(zmq.RCVBUF) > 0:
                    try:
                        self.socket.recv(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
            
            payload = self.socket.recv()
            
            msg = sharpa_hand_pb2.MocapKeypoints()
            msg.ParseFromString(payload)  
            
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 0
            
                
                
            
            return msg
                
        except Exception as e:
            print(f"[MocapKeypoints Receiver] Receive failed: {e}")
            return None

    def close(self):
        self.socket.close()
        self.context.term()

class HandActionReceiver:
    """HandAction message receiver"""
    def __init__(self, address="tcp://localhost:6666"):
        self.address = address
        
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        
        self.socket.setsockopt(zmq.RCVHWM, 1)  
        self.socket.setsockopt(zmq.LINGER, 0)    
        
        self.socket.connect(self.address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        print(f"[HandAction Receiver] Connected to {address}, no topic mode")
        print("[HandAction Receiver] Waiting for messages...")

    def receive_hand_action(self) -> Optional[sharpa_hand_pb2.HandAction]:
        """Receive HandAction message"""
        try:
            if self.socket.getsockopt(zmq.RCVBUF) > 8:  
                print("[HandAction Receiver] Warning: receive buffer full, dropping old messages")
                while self.socket.getsockopt(zmq.RCVBUF) > 0:
                    try:
                        self.socket.recv(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
            
            payload = self.socket.recv()
            
            msg = sharpa_hand_pb2.HandAction()
            msg.ParseFromString(payload)  
            
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 0
            
                
                
            
            return msg
                
        except Exception as e:
            print(f"[HandAction Receiver] Receive failed: {e}")
            return None

    def close(self):
        self.socket.close()
        self.context.term()

def main():
    print("=== Starting two receivers (no topic mode) ===")
    
    mocap_receiver = MocapKeypointsReceiver("tcp://192.168.10.222:2044")
    hand_receiver = HandActionReceiver("tcp://localhost:6666")
    
    try:
        while True:
            try:
                mocap_receiver.socket.setsockopt(zmq.RCVTIMEO, 1000)  
                mocap_receiver.receive_mocap_keypoints()
                hand_receiver.receive_hand_action()
            except zmq.Again:
                pass  
            
            
            time.sleep(0.01)  
            
    except KeyboardInterrupt:
        print("\n[Receiver] User interrupted")
    finally:
        mocap_receiver.close()
        hand_receiver.close()

if __name__ == "__main__":
    main() 