from input import Input

ctr = Input(index=0)

while 1:
    if ctr.available():
        # collect the event from the device, nonblocking
        e = ctr.event()
        while e:
            print(e)
            print([str(ax)+":"+str(e.axis(ax))+";" for ax in ctr.axis_names.values()])
            e = ctr.event()