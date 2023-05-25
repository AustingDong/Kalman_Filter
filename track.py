import numpy as np
from Kalman_Filter import KalmanFilter

class tracking_object:

    def __init__(self, label):
        self.state = False
        self.vis = False
        self.label = label
        self.threashold = 7
        self.mean_tracking = None
        self.covariance_tracking = None


    def judge(self, lst):
        '''
        judge(lst) handles the division problem happens in kf.
        '''
        if np.all(lst == [0, 0, 0]):
            return False
        return True


    def track(self, lst):
        '''
        track(lst) returns the posterior estimation and updates mean and covariance for
            this tracking object.

        parameters:
            lst: The node that you want to let your tracking object to track. 
        '''
        k_filter = KalmanFilter()
        mean_show = [0,0,0,0,0,0]
        if self.judge(lst):
            if not self.state:
                self.mean_tracking, self.covariance_tracking = k_filter.initiate(lst)
                self.state = True

            else:
                #update:
                self.mean_tracking, self.covariance_tracking = k_filter.update(self.mean_tracking, self.covariance_tracking, lst)

                #predict:
                self.mean_tracking, self.covariance_tracking = k_filter.predict(self.mean_tracking, self.covariance_tracking)
                mean_show = self.mean_tracking.copy()
        else:
            if self.state:
                self.mean_tracking, self.covariance_tracking = k_filter.predict(self.mean_tracking, self.covariance_tracking)
                mean_show = self.mean_tracking.copy()

        return mean_show
    

    def select(self, lst):
        '''
        select(lst) judges whether the tracking object should track this node.

        parameters:
            lst: The list of [x, y, z] that you want to judge.
        '''
        for i in range(len(lst)):
            if (np.abs(lst[i] - self.mean_tracking[i]) > self.threashold):
                return False
        return True

    def render(self):
        pass


def Track(frame, tracking_objects):
    for obj in tracking_objects:
        obj.vis = False

    for j in range(len(frame)):
        
        if tracking_objects == []:
            new_obj = tracking_object(f"car{len(tracking_objects) + 1}")
            new_obj.track(frame[j])
            tracking_objects.append(new_obj)

        else:
            found = False
            for obj in tracking_objects:
                if not obj.vis and obj.select(frame[j]):
                    obj.track(frame[j])
                    found = True

            if not found:
                new_obj = tracking_object(f"car{j + 1}")
                new_obj.track(frame[j])
                tracking_objects.append(new_obj)

    for obj in tracking_objects:
        print(obj.label, obj.mean_tracking)