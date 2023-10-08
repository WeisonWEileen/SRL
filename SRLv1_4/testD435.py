import D435

thread_D435 = D435.D435Thread()
thread_D435.run()
while True:
    depthframe = thread_D435.get_depth_frame()
    print(depthframe)