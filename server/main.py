import paho.mqtt.client as mqtt

broker = "localhost"  
port = 1883                     # Default MQTT port
controlTopic = "control/nav"  
dataTopic = "data/characters"

file="docs/file.txt"
with open(file, 'r') as file:
  text_content = file.read()
text_characters = list(text_content)
text_length = len(text_characters)

current_position=0

def send_data(client):
  global current_position
  data = text_characters[current_position].lower()
  if data == "\n":
    data = " "
  client.publish(dataTopic, data)

def update_position(navMsg):
  global current_position
  if navMsg == "10":
    if current_position != 0:
      current_position -= 1
  elif navMsg == "01":
    if current_position != text_length-1:
      current_position += 1
  else:
    pass

def callback(client, userdata, message):
  navMsg = message.payload.decode()
  update_position(navMsg)
  # print(f"Received message: {navMsg} on topic {message.topic}: {navMsg[0]} {navMsg[1]}")
  send_data(client)

client = mqtt.Client()
client.on_message = callback
client.connect(broker, port)
client.subscribe(controlTopic)

def main():
  send_data(client)
  client.loop_forever()

if __name__ == '__main__':
  main()