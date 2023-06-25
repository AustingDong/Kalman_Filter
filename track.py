import numpy as np
from Kalman_Filter import KalmanFilter

class tracking_object:

    def __init__(self, label):
        self.state = False
        self.vis = False
        self.label = label
        self.threshold = 0.5
        self.mean_tracking = None
        self.covariance_tracking = None

        # "dx, dy, dz" for generating bounding box
        self.size = []

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
        mean_show = [0,0,0,0,0,0,0,0]
        if self.judge(lst):
            if not self.state:
                self.mean_tracking, self.covariance_tracking = k_filter.initiate(lst)
                self.state = True

            else:

                #predict:
                self.mean_tracking, self.covariance_tracking = k_filter.predict(self.mean_tracking, self.covariance_tracking)


                #update:
                self.mean_tracking, self.covariance_tracking = k_filter.update(self.mean_tracking, self.covariance_tracking, lst)

                mean_show = self.mean_tracking.copy()
        else:
            if self.state:
                self.mean_tracking, self.covariance_tracking = k_filter.predict(self.mean_tracking, self.covariance_tracking)
                mean_show = self.mean_tracking.copy()

        return mean_show
    
    def prior_estimate(self):
        '''
        prior_estimate() produces prior estimation for selection.
        '''
        k_filter = KalmanFilter()
        prior_estimate_mean, prior_estimate_covariance = k_filter.predict(self.mean_tracking, self.covariance_tracking)
        return prior_estimate_mean
    

    #########################
    '''
    need to be change
    method: IoU
    '''
    def select(self, lst):
        '''
        select(lst) judges whether the tracking object should track this node.

        parameters:
            lst: The list of [x, y, z] that you want to judge.
        '''
        prior_estimation = self.prior_estimate()
        
        #########IoU############
        return self.IoU(lst) > self.threshold
        ########################

    ###################

    def bounding_box(self, lst):
        return [
            ...
        ]
        pass

    def IoU(self, lst):
        pass

    def render(self):
        pass


def Track(frame, tracking_objects, log=True):
    '''
    Track(frame, tracking_objects, log) consumes the current frame and track each objects.

    parameters:
        frame: current frame, a list of point[x, y, z].
        tracking_objects: a list of tracking_object.
        log: selection about whether output the informations of each tracking object or not.

    relations with code in paper:
        D -- frame
        T -- tracking_objects

    '''
    score_threshold = 0.5


    D_high = set()
    D_low = set()

    D_selected = set()
    T_selected = set()
    D_remain = set()
    T_remain = set()

    T_re_selected = set()
    T_re_remain = set()

    for d in frame:
        if d.score > score_threshold:
            D_high.add(d)
        else:
            D_low.add(d)

    #predict & associate D_high
    for t in tracking_objects:
        for d in D_high():
            if t.select(d):
                t.track(d)
                D_selected.add(d)
                T_selected.add(t)

    D_remain = D_high - D_low
    T_remain = tracking_objects - T_selected

    for t in T_remain:
        for d in D_low:
            if t.select(d):
                t.track(d)
                D_selected.add(d)
                T_re_selected.add(t)

    T_re_remain = T_remain - T_re_selected()

    #delete unmatched
    tracking_objects = tracking_objects - T_re_remain

    
    for d in D_remain:
        new_obj = tracking_object(f"car{len(tracking_objects) + 1}")
        new_obj.track(d)
        tracking_objects.add(new_obj)

    if log:
        for obj in tracking_objects:
            print(obj.label, obj.mean_tracking)

    return tracking_objects


    # for j in range(len(frame)):
        
    #     if tracking_objects == []:
    #         new_obj = tracking_object(f"car{len(tracking_objects) + 1}")
    #         new_obj.track(frame[j])
    #         tracking_objects.append(new_obj)

    #     else:
    #         found = False
    #         for obj in tracking_objects:
    #             if not obj.vis and obj.select(frame[j]):
    #                 obj.track(frame[j])
    #                 obj.vis = True
    #                 found = True
                    

    #         if not found:
    #             new_obj = tracking_object(f"car{j + 1}")
    #             new_obj.track(frame[j])
    #             tracking_objects.append(new_obj)

    

    
