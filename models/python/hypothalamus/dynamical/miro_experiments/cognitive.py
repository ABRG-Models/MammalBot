import cv2
import numpy as np

class AttentionSystem:
    def __init__( self, visual ):        
        self.x = 0
        self.visual = visual
        self.salient = False
        self.inhibited = True
        self.obstacle = False

    def inhibit( self ):
        self.inhibited = True
        self.salient = False

    def switch( self, x ):
        if not self.salient:
            self.x = x

    def update( self ):
        if self.inhibited:
            return

        images = self.visual.images
        m,n,_ = images[0].shape
        imgl = cv2.cvtColor(images[0], cv2.COLOR_RGB2GRAY) 
        imgr = cv2.cvtColor(images[0], cv2.COLOR_RGB2GRAY) 
        
        ml = np.mean(imgl[:,0:n/2])
        mr = np.mean(imgr[:,n/2:])
        mc = np.mean(imgl[:,n/2:])+np.mean(imgr[:,0:n/2])
        s = np.argsort([ml,mc,mr])

        if ml-mr > 30:
            self.x = -1
            self.salient = True
        elif mr-ml > 30:
            self.x = 1
            self.salient = True
        else:
            self.salient = False


class VisualPerception:
    def __init__(self):
        self.images = None

    def update( self, images ):
        self.images = images

    def isClose( self, images, r ):

        m1 = np.mean(images[0])
        m2 = np.mean(images[1])
        # print "m1: ", m1, ", m2: ", m2, ", size: ", r

        if r > 60:
            return True

        return False

    def getColor( self, image, x, y, r ):
        if image is None:
            return None

        m,n,_ = image.shape
        li = np.amax( [int(x)-r, 0] )
        ri = np.amin( [int(x)+r, n-1] )
        lj = np.amax( [int(y)-r, 0] )
        rj = np.amin( [int(y)+r, m-1] )
        # print "x: ", x, ", y: ", y, ", r: ", r
        # print "lj: ", lj
        # print "li: ", li
        image_s = image[lj:rj, li:ri, :]
        # print "Original shape: ", image.shape, ", sub shape: ", image_s.shape

        # cv2.imshow("Piece", image_s)
        # plt.hist(image_s[:,:,1].ravel(),256,[0,256])
        # cv2.waitKey(1)
        # image[0] = cv2.equalizeHist(image[0])
        # image[1] = cv2.equalizeHist(image[1])
        # image[2] = cv2.equalizeHist(image[2])
        boundaries = [
        ([17, 100, 15], [50, 230, 56]),
        ([86, 31, 4], [220, 88, 50])
        ]

        # for (lower, upper) in boundaries:
        lower, upper = boundaries[0]
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(image_s, lower, upper)
        output_g = cv2.bitwise_and(image_s, image_s, mask = mask)

        lower, upper = boundaries[1]
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(image_s, lower, upper)
        output_r = cv2.bitwise_and(image_s, image_s, mask = mask)

        mg = np.mean( image_s[:,:,1] )
        mr = np.mean( image_s[:,:,0] )
        mb = np.mean( image_s[:,:,2] )
        
        if mr > mg:
            return 'red'
        else:
            # print "mr: ", mr, ", mg: ", mg, ", mb: ", mb
            return 'green'

    def _locateCircle( self, img ):
        # img = cv2.resize(img, (320, 240))
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)       
        # Blur using 3 * 3 kernel. 
        gray_blurred = cv2.blur(gray, (3, 3)) 
        
        # Apply Hough transform on the blurred image. 
        detected_circles = cv2.HoughCircles(gray_blurred,  
                        cv2.HOUGH_GRADIENT, 1, 2, param1 = 90, 
                    param2 = 50, minRadius = 2, maxRadius = 200) 
        
        return detected_circles
    
    def _filterInput( self, img, circles, color ):
        if circles is not None: 
   
            detected_circles = np.uint16(np.around(circles)) 

            for pt in detected_circles[0, :]: 
                a, b, r = pt[0], pt[1], pt[2] 
                
                if self.getColor( img, a, b, r ) == color:
                    return [a, b, r]
        
        return None

    def locateCircle( self, images, color, draw = False ):
        if images is None:
            return None

        m,n,_ = images[0].shape
        # print m,n
        circle_left = self._locateCircle( images[0] )
        circle_right = self._locateCircle( images[1] )

        r_max = 0
        c_left = self._filterInput( images[0], circle_left, color )
        c_right = self._filterInput( images[1], circle_right, color )

        return c_left, c_right

    def getStimulusPositionAndSize(self, images, color):
        c_left, c_right = self.locateCircle( images, color )
        stim_x = None
        stim_y = None
        object_size = 0

        if c_left is None and c_right is None:
            return stim_x, stim_y, object_size

        n,m,_ = images[0].shape
      
        sigma = m/3.0
        gl = lambda x: np.exp(-((float(x) - 0)**2)/(2*sigma**2))
        gc = lambda x: np.exp(-((-float(x) + m)**2)/(2*sigma**2))
        pl_l, pl_c, pr_c, pr_r = [0]*4
        y_pos = n/2.0
        # left image
        if c_left is not None:
            pl_l = gl( c_left[0] )
            pl_c = gc( c_left[0] )
            object_size = c_left[2]
            y_pos = c_left[1]
        # right image
        if c_right is not None:
            pr_c = gl( c_right[0] )
            pr_r = gc( c_right[0] )
            object_size = c_right[2]
            y_pos = c_right[1]

        if pl_l > pl_c:
            stim_x = -1
            
        elif pr_r > pr_c:
            stim_x = 1            
        else:
            stim_x = 0

        if y_pos < n/3.0:
            stim_y = -1
        elif y_pos > 2*n/3.0:
            stim_y = 1
        else:
            stim_y = 0

        return stim_x, stim_y, object_size

    def plot( self ):
        pass