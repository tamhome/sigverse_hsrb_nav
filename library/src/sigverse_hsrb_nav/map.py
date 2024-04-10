#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from tamlib.utils import Logger
from nav_msgs.srv import LoadMap, LoadMapRequest


class MapServerUtils(Logger):
    def __init__(self):
        """
        """
        Logger.__init__(self)

        # サービスのクライアントを作成
        self.change_map_service = rospy.ServiceProxy('/change_map', LoadMap)

    def change_map(self, map_url: str) -> int:
        """読み込みマップを変更する
        Args:
            map_url: map.yamlのパスを指定
        Returns:
            サーバからの結果を返す
        """
        try:
            req = LoadMapRequest()
            req.map_url = map_url  # サービス定義に合わせて属性を設定

            # サービスを呼び出し
            _ = self.change_map_service(req)
            return True

        except rospy.ServiceException as e:
            self.logwarn("cannot change map")
            self.logwarn(str(e))
            return False


if __name__ == "__main__":
    cls = MapServerUtils()
    map_url = "temp/map.yaml"
    cls.change_map(map_url=map_url)
