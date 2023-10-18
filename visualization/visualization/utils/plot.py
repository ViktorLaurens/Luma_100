import matplotlib.pyplot as plt

fig = plt.figure(figsize=(8, 8), dpi=80)

repeat_length = 50

def plot_setting():
    global repeat_length
    fig.canvas.set_window_title('Messages received')
    repeat_length = 50
    plt.xlim(0, repeat_length)
    plt.ylim(0, 10)
    plt.xlabel('messages in time')
    plt.ylabel('agent_id')

prev_x = 0
prev_y = 0

def draw_plot(x, y):
    global repeat_length, prev_x, prev_y

    # make the axis move dynamically with the data
    if(x > repeat_length):
        plt.xlim(x - repeat_length, x)
    else:
        plt.xlim(0, repeat_length)

    plt.plot([prev_x, x], [prev_y, y],  'ob-')
    plt.pause(0.01)

    prev_x = x
    prev_y = y 

'''
def update_plot(self):
    self.ax.clear()  # Clear the current plot
    
    # Plot sent messages
    ax.scatter(self.sent_x, self.sent_y, marker='o', color='b', label='Sent')
    for i, txt in enumerate(self.sent_ids):
        self.ax.annotate(txt, (self.sent_x[i], self.sent_y[i]))

    # Plot received messages
    self.ax.scatter(self.received_x, self.received_y, marker='x', color='r', label='Received')
    for i, txt in enumerate(self.received_ids):
        self.ax.annotate(txt, (self.received_x[i], self.received_y[i]))
    
    self.ax.set_xlim(0, 50)
    self.ax.set_ylim(0, 3)
    plt.xlabel('message order')
    plt.ylabel('x and y')
    
    plt.legend()
    plt.draw()
    plt.pause(0.01)  # Allows the plot to update
'''