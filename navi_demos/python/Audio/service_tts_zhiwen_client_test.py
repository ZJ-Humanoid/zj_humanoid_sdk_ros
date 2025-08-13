#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from navi_types.srv import Audio_TTS, Audio_TTSRequest
import sys
import asyncio
import json
import logging

from typing import Callable, Optional
import websockets
from websockets.client import WebSocketClientProtocol

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# WebSocket服务器地址
WS_SERVER_URL = "ws://127.0.0.1:12393/client-ws"


class WebSocketClient:
    def __init__(self, uri: str, message_handler: Callable[[str], None]):
        """Initialize WebSocket client.
        
        Args:
            uri: WebSocket server URI
            message_handler: Callback function to handle received messages
        """
        self.uri = uri
        self.message_handler = message_handler
        self.websocket: Optional[WebSocketClientProtocol] = None
        self.running = False
        self.connected = False
        self.reconnect_attempts = 0
        self.max_reconnect_delay = 30
        self.logger = logging.getLogger(__name__)

    def start(self):
        """Start the WebSocket client in the event loop."""
        self.running = True
        asyncio.run(self.run())

    def stop(self):
        """Stop the WebSocket client."""
        self.running = False
        if self.websocket:
            asyncio.create_task(self.websocket.close())

    async def send(self, message: str):
        """Send a message to the WebSocket server.
        
        Args:
            message: Message to send
        """
        try:
            if self.connected and self.websocket:
                await self.websocket.send(message)
            else:
                self.logger.error("Connection is not open")
                rospy.logerr("Connection is not open")
        except Exception as e:
            self.logger.error(f"Error sending message: {str(e)}")
            rospy.logerr(f"Error sending message: {str(e)}")

    async def run(self):
        """Main loop for the WebSocket client."""
        while self.running:
            try:
                async with websockets.connect(self.uri) as websocket:
                    self.websocket = websocket
                    self.connected = True
                    self.reconnect_attempts = 0
                    self.logger.info("Connected to WebSocket server")
                    rospy.loginfo("Connected to WebSocket server")

                    try:
                        async for message in websocket:
                            if self.message_handler:
                                self.message_handler(message)
                    except websockets.ConnectionClosed:
                        self.logger.info("WebSocket connection closed")
                        rospy.loginfo("WebSocket connection closed")
                    finally:
                        self.connected = False
                        self.websocket = None

            except Exception as e:
                self.logger.error(f"Connection error: {str(e)}")
                rospy.logerr(f"Connection error: {str(e)}")
                delay = self._get_reconnect_delay()
                self.logger.info(f"Retrying in {delay} seconds...")
                rospy.loginfo(f"Retrying in {delay} seconds...")
                await asyncio.sleep(delay)

    def _get_reconnect_delay(self) -> float:
        """Calculate reconnect delay using exponential backoff.
        
        Returns:
            Delay in seconds
        """
        delay = min(2 ** self.reconnect_attempts, self.max_reconnect_delay)
        self.reconnect_attempts += 1
        return delay


def message_handler(message: str, tts_proxy):
    """处理从WebSocket服务器接收到的消息"""
    try:
        data = json.loads(message)
        if isinstance(data, dict):  # 确保 data 是一个字典
            rospy.loginfo(f"Keys in data: {list(data.keys())}")

        
        if data.get('type') == 'audio' and 'display_text' in data:
            rospy.loginfo(f"回复: {data['slice_length']}")

            rsp_text = data['display_text']['text']
            rospy.loginfo(f"回复: {rsp_text}")
            # request = Audio_TTSRequest(text=rsp_text, isPlay=True)
            # response = tts_proxy(request)
            # if response.status == 200:
            #     rospy.loginfo(f"TTS 服务调用成功: {response.file_path}")
            # else:
            #     rospy.logerr("TTS 服务调用失败")

    except json.JSONDecodeError:
        pass

async def main():
    """
    TTS 服务
    """
    # 初始化 ROS 节点
    rospy.init_node('tts_service_test_client', anonymous=True)
    rospy.loginfo("TTS 服务测试客户端已启动")
    
    # 等待 TTS 服务可用
    rospy.loginfo("等待 TTS 服务...")
    rospy.wait_for_service('Audio/tts_service')

    try:
        # 创建服务代理
        tts_proxy = rospy.ServiceProxy('Audio/tts_service',Audio_TTS)

         # 智问 ws 客户端
        ws_client = WebSocketClient(WS_SERVER_URL, lambda msg: message_handler(msg, tts_proxy))
        
        # 启动客户端
        ws_client.running = True
        run_task = asyncio.create_task(ws_client.run())
        
        # 等待连接建立
        await asyncio.sleep(1)

        # 测试用例1：发送文本输入
        test_message1 = {
            "type": "text-input",
            "text": "介绍一下中控"
        }
        rospy.loginfo(f"Sending test message 1: {json.dumps(test_message1, ensure_ascii=False)}")
        await ws_client.send(json.dumps(test_message1))
        
        # 等待响应
        await asyncio.sleep(5)
            
    except rospy.ServiceException as e:
        rospy.logerr(f"调用 TTS 服务失败: {e}")
    finally:
        # 停止客户端
        ws_client.stop()
        await run_task


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
    except Exception as e:
        logger.error(f"An error occurred: {str(e)}")
    finally:
        logger.info("Test completed")