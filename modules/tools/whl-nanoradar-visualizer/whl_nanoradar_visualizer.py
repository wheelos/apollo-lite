#!/usr/bin/env python3
"""whl-nanoradar-visualizer
"""

import functools
import json
import pathlib
import asyncio
import aiohttp
import aiohttp.web
import click
from cyber.python.cyber_py3 import cyber
from modules.common_msgs.sensor_msgs import nano_radar_pb2


class NanoRadarDataReceiver:
    """NanoRadarDataReceiver
    """

    class DataConverter:
        """DataConverter
        """

        def __init__(self, data: nano_radar_pb2.NanoRadar):
            self.data = data

        def __call__(self, *args, **kwargs):
            return self.data

        def CopyFrom(self, data: nano_radar_pb2.NanoRadar):
            """CopyFrom
            """
            self.data.CopyFrom(data)

        @property
        def obstacles(self):
            """obstacles
            """
            ret = []
            for obj in self.data.contiobs:
                ret.append({
                    'id': obj.obstacle_id,
                    'x': obj.lateral_dist,
                    'y': obj.longitude_dist,
                    'z': 0.0,
                    'vx': obj.lateral_vel,
                    'vy': obj.longitude_vel,
                    'vz': 0.0,
                    'v': obj.obstacle_vel,
                    'rcs': obj.rcs,
                    'angle': obj.oritation_angle,
                    'type': obj.obstacle_class,
                    'dynprop': obj.dynprop,
                    'range': obj.obstacle_range,
                })
            return ret

        @property
        def scan_area(self):
            """scan_area
            """
            if self.data.radar_region_state:
                return {
                    'p1': {
                        'x': self.data.radar_region_state.point1_lateral,
                        'y': self.data.radar_region_state.point1_longitude,
                    },
                    'p2': {
                        'x': self.data.radar_region_state.point2_lateral,
                        'y': self.data.radar_region_state.point2_longitude,
                    },
                }
            return None

    def __init__(self, channels):
        """__init__
        """
        self.node = cyber.Node("whl-nanorader-visualizer")
        self.nanoradar_data = nano_radar_pb2.NanoRadar()
        self.channel_data_map = {}

        if channels:
            for channel in channels:
                self.channel_data_map[channel] = self.DataConverter(
                    nano_radar_pb2.NanoRadar())
                self.node.create_reader(
                    channel, nano_radar_pb2.NanoRadar,
                    functools.partial(self.on_nanoradar_data_received,
                                      channel))

    def on_nanoradar_data_received(self, channel, data):
        """on_nanoradar_data_received
        """
        if channel not in self.channel_data_map:
            raise ValueError(f"Channel {channel} not registered.")
        self.channel_data_map[channel].CopyFrom(data)

    def to_dict(self):
        """to_dict
        """
        ret = {}
        for channel, converter in self.channel_data_map.items():
            ret[channel] = {
                'obstacles': converter.obstacles,
                'scan_area': converter.scan_area,
            }
        return ret


async def handle_static(request):
    """serve index.html
    """
    return aiohttp.web.FileResponse(
        pathlib.Path(__file__).parent / 'index.html')


async def handle_websocket(request):
    """handle_websocket
    """
    ws = aiohttp.web.WebSocketResponse()
    await ws.prepare(request)

    while True:
        data = request.app['receiver'].to_dict()
        # print(data)
        await ws.send_str(json.dumps(data))
        await asyncio.sleep(0.1)  # 100ms更新频率

    return ws


@click.command()
@click.option('-c',
              '--channel',
              default=['/apollo/sensor/nanoradar/front'],
              multiple=True,
              help='NanoRadar data channel')
@click.option('-p', '--port', default=8765, help='server port')
def main(channel, port):
    """whl-nanoradar-visualizer
    """
    cyber.init()

    receiver = NanoRadarDataReceiver(channel)

    app = aiohttp.web.Application()
    app['receiver'] = receiver
    app.router.add_get('/', handle_static)
    app.router.add_get('/ws', handle_websocket)
    aiohttp.web.run_app(app, port=port)


if __name__ == '__main__':
    main()
