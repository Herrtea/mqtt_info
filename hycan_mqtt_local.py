from paho.mqtt import client as mqtt_client
import time,json,uuid
import datetime as dt

HOST = 'iot.dfrobot.com.cn'
# HOST = '127.0.0.1'
PORT = 1883
keepalive = 60
client_id = 'avp_infra_ID2'

sub_topic = '8gTVk3h7R'
pub_topic = '8gTVk3h7R'

# sub_topic = 'test/#'
# pub_topic1 = 'test/1'
# pub_topic2 = 'test/2'

username = '3BM8z3h7R'
password = 'qfGUz3hngz'
# username = 'avp_infra'
# password = '123456'

class Mqtt(object):
    # _sent_count = 0

    # header
    seq = 0
    timestamp = 0
    sender = 'IPS'
    model_version = '1.0.0'

    # payload
    id = 0
    status = 0
    x = 0
    y = 0
    yaw = 0
    length = 1.2
    width = 1.0
    speed = 0.1

    def __init__(self, host, port=1883, username='', password='', client_id=None, keepalive=60, *argv, **kwargv):
        super(Mqtt,self).__init__()
        if client_id == None:
            client_id = uuid.UUID(int = uuid.getnode()).hex[-12:]
        self.client = mqtt_client.Client(client_id,clean_session=None,**kwargv)
        self.host = host
        self.port = port
        self.keepalive = keepalive
        self.client.on_connect = self.on_connect
        self.client.on_publish = self.on_publish
        self.client.on_message = self.on_message
        self.client.on_publish = self.on_publish
        self.client.on_subscribe = self.on_subscribe
        self.client.on_unsubscribe = self.on_unsubscribe
        self.client.on_disconnect = self.on_disconnect
        self.client.username_pw_set(username=username, password=password)
        self.client.reconnect_delay_set(min_delay=1, max_delay=120)
        self.client.subscribe_callback = self.subscribe_cb
        self.client.connect_fail_callback = self.connect_fail_cb
        self.client.disconnect_callback = self.disconnect_cb
        self.is_reconnect = True

    def disconnect_cb(self):
        if self.is_reconnect:
            self.client.reconnect()
        print('------mqtt disconnect_cb------')

    def connect_fail_cb(self):
        print('------mqtt connect_fail_cb------')

    # mqtt连接服务器
    def connect(self):
        self.client.connect(self.host,self.port,self.keepalive)

    # mqtt连接回调
    def on_connect(self, client, userdata, flags, rc):
        print('------mqtt connect------')
        print('Connect result:'+mqtt_client.connack_string(rc))

    # mqtt发布成功回调
    def on_publish(self, client, userdata, mid):
        print('------mqtt publish------')
        print('mid:'+str(mid) + ' Running...')

    # mqtt接收到成功消息回调
    def on_message(self, client, userdata, msg):
        print('------mqtt message------')
        now = dt.datetime.now()
        print(f'Recv topic:{msg.topic}  {now}  data:'+str(msg.payload,'utf-8'))
        # self._sent_count += 1
        # pub_msg = 'msg count:%d' % self._sent_count
        time_sleep = 1

        self.seq += 1
        # self.timestamp = time.time()
        now = dt.datetime.now()
        self.timestamp = str(dt.datetime.now())[:-3]

        self.x += 0.1
        self.y += 0.2
        self.yaw += 0.05

        pub_msg = {
            'header': [
                {
                    'seq': self.seq,
                    'time': self.timestamp,
                    'sender': self.sender,
                    'model_version': self.model_version
                }
            ],

            'payload': [
                    {
                        'id': self.id,
                        'status': self.status,
                        'y': self.y,
                        'yaw': self.yaw,
                        'l': self.length,
                        'w': self.width,
                        'speed': self.speed
                     }
            ]
        }
        self.client.publish(topic=pub_topic, payload=json.dumps(pub_msg))
        # time.sleep(0.05)
        # self.client.publish(topic=pub_topic1, payload=json.dumps(pub_msg))
        # self.client.publish(topic=pub_topic2, payload=json.dumps(pub_msg))

    # mqtt订阅成功回调
    def on_subscribe(self, client, userdata, mid, granted_qos):
        print('------mqtt subscribe------')
        print('On subscribe: qos = %d'%granted_qos)

    # mqtt取消订阅成功回调
    def on_unsubscribe(self, client, userdata, mid, granted_qos):
        print('------mqtt unsubscribe------')
        print('On unsubscribe: qos = %d'%granted_qos)

    # mqtt成功断开连接回调
    def on_disconnect(self, client, userdata, rc):
        print('------mqtt disconnect------')
        print('Disconnect result:'+mqtt_client.connack_string(rc))

    def subscribe_cb(self):
        print('------mqtt subscribe_cb------')

    def close(self):
        self.client.reconnect()


def main():

    mqtt = Mqtt(host=HOST,port=PORT,
            username=username,password=password,client_id=client_id,keepalive=keepalive)

    will_MSG = {
        'ID':f'{client_id}',
        'stat':'Offline'
    }
    mqtt.connect()

    # mqtt.client.will_set(pub_topic,payload=json.dumps(will_MSG),qos=0,retain=True)
    mqtt.client.subscribe(topic=sub_topic,qos=0)
    mqtt.client.loop_forever()


if __name__ == '__main__':

    main()