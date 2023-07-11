import os
import numpy as np
import open3d as o3d
import matplotlib
import matplotlib.pyplot as plt
from scipy import signal
import cv2 as cv
from datetime import datetime
import statistics as st

from skimage import measure

from library.pcd_to_2d import to_2d
from library.convert_file_type import img_to_pcd
import library.preprocessing as prep
from skimage.measure import regionprops

from time import sleep

def preprocess_pcd(pcd: o3d.geometry.PointCloud, thickness: float, top, bottom):
    """
    Reads in a csv point cloud.
    Then processes it to a 2d depth image.
    Writes the 2d depth image to a file with the input_file name in the data/preprocessing_output folder

    :param pcd: (o3d.geometry.PointCloud) input pointcloud
    :param thickness: (float) thickness of the steelplate
    :returns (o3d.geometry.PointCloud) processed point cloud
    """
    xyz = np.asarray(pcd.points)
    
    # filtered pcd which is used for the preprocessing
    pcd_sel = pcd.select_by_index(np.where(xyz[:, 2] >= 0)[0])
    o3d.visualization.draw_geometries([pcd_sel], window_name='pcd_sel', mesh_show_wireframe=True)
    # return pcd_sel
    print(f'thickness = {thickness}')
    # Filter out all values too far away from the steel plate z-axis
    pcd_filtered = prep.filter_out_steel_plate(pcd_sel, thickness, top, bottom)
    o3d.visualization.draw_geometries([pcd_filtered], window_name='pcd_filtered')

    # Remove the outliers from the filtered pcd
    pcd_inliers = prep.remove_outliers(pcd_filtered)
    o3d.visualization.draw_geometries([pcd_inliers], window_name='pcd_inliers', mesh_show_wireframe=True)

    # Get the axis aligned bounding box
    aabb = prep.get_bounding_box(pcd_inliers)

    # Crop the picture with the bbox
    pcd = pcd_inliers.crop(aabb)

    return pcd


class PointCloud:
    def __init__(self, file_name, file_type='-bin.pcd', verbose=False):
        self.verbose = verbose
        self.MM_PER_DIST = os.getenv("mm_per_dist")

        matplotlib.use('GTK3Agg')

        if file_type == ".pcd" or file_type == "-bin.pcd":
            self.file_path = os.path.realpath('') + os.getenv("pcd_path") + file_name + file_type
        elif file_type == ".png":
            # todo:: check if pcd file already exists

            # if find_pcd(file_name):
            #     self.file_path = os.path.realpath('.') + os.getenv("pcd_path") + file_name + ".pcd"
            # if find_compressed_pcd(file_name):
            #     self.file_path = os.path.realpath('.') + os.getenv("pcd_path") + file_name + "-bin.pcd"
            # else:
            self.file_path = img_to_pcd(file_name + file_type)
        else:
            raise Exception(f"Error: unrecognized file type given: {file_type}")
        
    def isolate_plate(self, show):
        # get pcd
        pcd = o3d.io.read_point_cloud(self.file_path)
        if show: self.show(pcd, "from_png")

        # crop to one side of the flipper
        pcd = self.crop_start_side(pcd)
        if show: self.show(pcd, "crop_start_side")

        # crop everything below flipper plate
        xyz = np.asarray(pcd.points)
        planes = self.segment_planes(xyz)
        print(planes['bottom'])
        pcd = pcd.select_by_index(np.where(xyz[:, 2] >= 520)[0])
        if show: self.show(pcd, "crop everything below bottom plane")

        # remove outliers
        down_pcd = pcd.voxel_down_sample(voxel_size=0.1)
        cl, ind = down_pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
        pcd = down_pcd.select_by_index(ind)
        # xyz = np.asarray(pcd.points)
        # pcd = pcd.select_by_index(np.where(xyz[:, 2] <= 600)[0])
        if show: self.show(pcd, "remove outliers")

        xyz = np.asarray(pcd.points)

        x = xyz[:,0]
        y = xyz[:,1]
        z = xyz[:,2]

        print(f"x[0]: {x[0]}")
        print(f"x[len(x)-1]: {x[len(x)-1]}")

        print(f"y[0]: {y[0]}")
        print(f"y[len(y)-1]: {y[len(y)-1]}")

        print(f"z[0]: {z[0]}")
        print(f"z[len(z)-1]: {z[len(z)-1]}")

        slope = (y[len(y)-1] - y[0])/(z[len(z)-1]-z[0])

        print(f"slope: {slope}")

        crop_pcd = self.crop_slope(pcd)
        if show: self.show(crop_pcd,"slope")
        xyz = np.asarray(crop_pcd.points)
        x = xyz[:,0]
        y = xyz[:,1]
        z = xyz[:,2]

        slope1 = (y[len(y)-1] - y[0])/(z[len(z)-1]-z[0])
        slope2 = ((z[len(z)-1]-z[0]/y[len(y)-1] - y[0]))

        print(f"slope: {slope1}")
        print(f"slope: {slope2}")

        import math as m
        xyz = np.asarray(pcd.points)
        
        xyz = xyz * self.Rx(m.radians(90.4))
        pcd.points = o3d.utility.Vector3dVector(xyz)

        x = xyz[:,0]
        y = xyz[:,1]
        z = xyz[:,2]

        slope1 = (y[len(y)-1] - y[0])/(z[len(z)-1]-z[0])
        print(f"slope: {slope1}")

        xyz = np.asarray(pcd.points)
        # planes = self.segment_planes(xyz)

       

        x = xyz[:,0]
        y = xyz[:,1]
        z = xyz[:,2]

        print(f"x[0]: {x[0]}")
        print(f"x[len(x)-1]: {x[len(x)-1]}")

        print(f"y[0]: {y[0]}")
        print(f"y[len(y)-1]: {y[len(y)-1]}")

        print(f"z[0]: {z[0]}")
        print(f"z[len(z)-1]: {z[len(z)-1]}")

        if show: self.show(pcd, "rotated?")



        # xyz = np.asarray(pcd.points)
        # planes = self.segment_planes(xyz)

        # # Get the axis aligned bounding box
        # aabb = prep.get_bounding_box(pcd)

        # # Crop the picture with the bbox
        # pcd = pcd.crop(aabb)
        # self.show(pcd, "bounding box")
        
        # xyz = np.asarray(pcd.points)
        # print(xyz)

        # z = xyz[:, 2]
        # min_max_dist = max(z) - min(z)
        # bin_size_mm = 0.5
        # bin_size = bin_size_mm / float(self.MM_PER_DIST)

        # print(f"thickness: {min_max_dist}")
        # print(f"thickness: {((max(z) * bin_size_mm) - (min(z) * bin_size_mm))}")

        # print(xyz[:,2])

        # st_dev = st.stdev(z)
        # z[z < min_max_dist - st_dev * 1.1] = 500
        # z[z > min_max_dist + st_dev * 1.5] = 500

        # # We recreate the pcd with the filtered values.
        # # This creates a pcd only with the steel plate points at the mode height
        # # This also leaves some points which are at the height of the steel plate,
        # # but are not part of the plate. These we will remove later with outlier removal.
        # pcd.points = o3d.utility.Vector3dVector(xyz)
        # # points = np.asarray(pcd.points)
        # # pcd = pcd.select_by_index(np.where(points[:, 2] > min_max_dist - 2)[0])
        # self.show(pcd)


        pcd = pcd.select_by_index(np.where(xyz[:, 1] >= 533)[0])
        self.show(pcd, 'just plate?')

        # remove outliers
        down_pcd = pcd.voxel_down_sample(voxel_size=0.1)
        cl, ind = down_pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
        pcd = down_pcd.select_by_index(ind)
        # xyz = np.asarray(pcd.points)
        # pcd = pcd.select_by_index(np.where(xyz[:, 2] <= 600)[0])
        if show: self.show(pcd, "remove outliers")

    def Rx(self, theta):
        import math as m
        return np.matrix([[ 1 , 0            , 0             ],
                          [ 0 , m.cos(theta) , -m.sin(theta) ],
                          [ 0 , m.sin(theta) , m.cos(theta)  ]])

    def Ry(self, theta):
        import math as m
        return np.matrix([[ m.cos(theta) , 0 , m.sin(theta) ],
                          [ 0            , 1 , 0            ],
                          [ -m.sin(theta), 0 , m.cos(theta) ]])

    def Rz(self, theta):
        import math as m
        return np.matrix([[ m.cos(theta) , -m.sin(theta), 0 ],
                          [ m.sin(theta) , m.cos(theta) , 0 ],
                          [ 0            , 0            , 1 ]])

    def segment_planes(self, xyz):
        # get z values
        z = xyz[:, 2]

        # size of the bins in the histogram function, defines the 'resolution' of the peak finding
        bin_size_mm = 0.5
        # peak dist bins is the minimum distance in bins between two recognized peaks
        peak_dist_bins = 5

        min_max_dist = max(z) - min(z)
        bin_size = bin_size_mm / float(self.MM_PER_DIST)
        bin_amt = round(min_max_dist / bin_size)

        z_hist = np.histogram(z, bins=bin_amt)

        # prominence set manually, might need tweaking/alternative
        peaks = signal.find_peaks(z_hist[0], distance=peak_dist_bins, prominence=6000)
        # get the actual peak indices
        peaks = peaks[0]
        print(f"peaks: {peaks}")
        
        return {
            # 'top' : peaks[3] * bin_size + min(z),
            'bottom' : peaks[0] * bin_size + min(z)
        }
    
    def crop_z(self, pcd, bottom):
        xyz = np.asarray(pcd.points)
        return pcd.select_by_index(np.where(xyz[:, 2] >= bottom)[0])
        xyz = np.asarray(pcd.points)
        return pcd.select_by_index(np.where(xyz[:, 2] <= top)[0])
    

    def crop_start_side(self, pcd):
        xyz = np.asarray(pcd.points)
        return pcd.select_by_index(np.where(xyz[:, 1] >= 350)[0])
    
    def crop_deliver_side(self, pcd):
        xyz = np.asarray(pcd.points)
        
        return pcd.select_by_index(np.where(xyz[:, 1] <= 260)[0])
    
    def crop_slope(self, pcd):
        xyz = np.asarray(pcd.points)
        pcd = pcd.select_by_index(np.where(xyz[:, 0] >= 214)[0])
        xyz = np.asarray(pcd.points)
        return pcd.select_by_index(np.where(xyz[:, 0] <= 215)[0])
        
    def extract_features(self):
        """
            Generate features from a PointCloud
            :return dict containing the features extracted
            """
        _start_time = datetime.now()
        # read in the pointcloud
        pcd = o3d.io.read_point_cloud(self.file_path)

        xyz = np.asarray(pcd.points)
        _end_time = datetime.now()
        if self.verbose:
            print(f"['extract_features: point cloud reading'] = {_end_time} - {_start_time}")

        # extract the horizontal planes
        _start_time = datetime.now()
        plate_planes = self.extract_plate_planes(pcd)
        _end_time = datetime.now()
        if self.verbose:
            print(f"['extract_features: extract horizontal planes'] = {_end_time} - {_start_time}")

        # set the bottom of the plate to z=0
        _start_time = datetime.now()

        # xyz[:, 2] = xyz[:, 2] - plate_planes['bottom']
        # pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd = pcd.select_by_index(np.where(xyz[:, 2] >= plate_planes['bottom'])[0])
        xyz = np.asarray(pcd.points)
        pcd = pcd.select_by_index(np.where(xyz[:, 2] <= plate_planes['top'])[0])
        xyz = np.asarray(pcd.points)

        _end_time = datetime.now()
        if self.verbose:
            print(f"['extract_features: plate bottom recognition'] = {_end_time} - {_start_time}")

        # calculate thickness of plate
        _start_time = datetime.now()
        thickness = plate_planes['top'] - plate_planes['bottom']
        _end_time = datetime.now()
        if self.verbose:
            print(f"['extract_features: thickness calculation'] = {_end_time} - {_start_time}")

        # apply preprocessing
        _start_time = datetime.now()
        pcd = preprocess_pcd(pcd, thickness, plate_planes['top'], plate_planes['bottom'])
        
        xyz = np.asarray(pcd.points)

        # get z values
        z = xyz[:, 2]

        # size of the bins in the histogram function, defines the 'resolution' of the peak finding
        bin_size_mm = 0.5
        # peak dist bins is the minimum distance in bins between two recognized peaks
        peak_dist_bins = 5

        print(f"z: {z}")
        min_max_dist = max(z) - min(z)
        bin_size = bin_size_mm / float(self.MM_PER_DIST)
        bin_amt = round(min_max_dist / bin_size)

        z_hist = np.histogram(z, bins=bin_amt)

        print(f"z_hist: {z_hist[0]}")

        # prominence set manually, might need tweaking/alternative
        peaks = signal.find_peaks(z_hist[0], distance=peak_dist_bins, prominence=6000)
        # get the actual peak indices
        peaks = peaks[0]

        # With this code we can visualize the histogram and the peaks
        # plt.plot(z_hist[0], bin_amt)
        # plt.
        plt.plot(peaks, z_hist[0][peaks], "x")
        plt.show()

        o3d.visualization.draw_geometries([pcd])

        plate_top_pcd = o3d.geometry.PointCloud()
        xyz = np.asarray(pcd.points)

        top_xyz = xyz[xyz[:, 2] > (plate_planes['top'] - plate_planes['bottom'])]

        plate_top_pcd.points = o3d.utility.Vector3dVector(top_xyz)

        size = self.get_length_width(plate_top_pcd)
        _end_time = datetime.now()
        if self.verbose:
            print(f"['extract_features: preprocessing'] = {_end_time} - {_start_time}")

        # save the 2d image
        _start_time = datetime.now()
        to_2d(pcd, self.file_path)
        _end_time = datetime.now()
        if self.verbose:
            print(f"['extract_features: save to 2D image'] = {_end_time} - {_start_time}")

        return {'thickness': thickness,
                'length': size['length'],
                'width': size['width']}

    def extract_plate_planes(self, pcd: o3d.geometry.PointCloud):
        """
        Extract horizontal planes of the plate in a given pointcloud

        :param pcd: (o3d.geometry.PointCloud) the pointcloud to process
        :return: dict containing 'top' and 'bottom' keys representing these parts of the plate
        """
        xyz = np.asarray(pcd.points)

        # get z values
        z = xyz[:, 2]

        # size of the bins in the histogram function, defines the 'resolution' of the peak finding
        bin_size_mm = 0.5
        # peak dist bins is the minimum distance in bins between two recognized peaks
        peak_dist_bins = 5

        print(f"z: {z}")
        min_max_dist = max(z) - min(z)
        bin_size = bin_size_mm / float(self.MM_PER_DIST)
        bin_amt = round(min_max_dist / bin_size)

        z_hist = np.histogram(z, bins=bin_amt)

        print(f"z_hist: {z_hist[0]}")

        # prominence set manually, might need tweaking/alternative
        peaks = signal.find_peaks(z_hist[0], distance=peak_dist_bins, prominence=6000)
        # get the actual peak indices
        peaks = peaks[0]

        # With this code we can visualize the histogram and the peaks
        # plt.plot(z_hist[0], z_hist[1])
        # plt.plot(z_hist[0], bin_amt)
        plt.plot(peaks, z_hist[0][peaks], "x")
        plt.show()

        # this part might not work so great in all situations
        conveyor = peaks[0]
        bottom = peaks[1]
        top = peaks[-1]

        print(f"peaks: {peaks}")
        print(f"top: {peaks[2]}")
        print(f"bottom: {peaks[1]}")
        print(f"conveyor: {peaks[0]}")

        # We add the min(z) back to the peaks, so that the peaks from the histogram align with the peaks in the pcd
        return {
            'conveyor': conveyor * bin_size + min(z),
            'bottom': bottom * bin_size + min(z),
            'top': top * bin_size + min(z),
        }

    def get_position(self, pcd):
        """get the length and width based off a pcd only containing points belonging to the plate using minimum bounding box

        Args:
            pcd (PointCloud): a pointcloud containing only points belonging to the plate

        Returns:
            dict: dictionary with 'length' and 'width' attributes
        """

        xyz = np.asarray(pcd.points)
        xy = xyz[:, (0, 1)].astype(np.float32)

        # get the corners of the counding box
        _, size, _ = cv.minAreaRect(xy)

        length = max(size) * float(self.MM_PER_DIST)
        width = min(size) * float(self.MM_PER_DIST)

        return {'x': x, 'y': y}

    def get_length_width(self, pcd):
        """get the length and width based off a pcd only containing points belonging to the plate using minimum bounding box

        Args:
            pcd (PointCloud): a pointcloud containing only points belonging to the plate

        Returns:
            dict: dictionary with 'length' and 'width' attributes
        """

        xyz = np.asarray(pcd.points)
        xy = xyz[:, (0, 1)].astype(np.float32)

        # get the corners of the counding box
        _, size, _ = cv.minAreaRect(xy)

        length = max(size) * float(self.MM_PER_DIST)
        width = min(size) * float(self.MM_PER_DIST)

        return {'length': length, 'width': width}

    def segment_image(self, image, MM_PER_PIX):
        """
        Takes in an image and segments all holes in the plates. for each of these segments is then determined
        whether the segment is a hole or an interior contour. These are returned in an array.

        NOTE: SO FAR ONLY CLASSIFIES HOLES

        :param image: an RGB numpy array representing an image as returned by io.imread() from skimage
        :returns a list of holes and/or interior contours
        """

        # minimum size for a segmented area (to ignore small holes from image creation)
        MIN_AREA = 50

        # select the pixels belonging to the plate by thresholding away all white pixels
        plate_pixels = image > 0.1

        # set pixels belonging to the plate to region 0, the background to region 1
        regions = np.where(plate_pixels, 0, 1)

        # we use skimage measure label function to seperate connected regions into different integer values
        labeled, num_labels = measure.label(regions, return_num=True)
        holes = []

        # get regionprops for all regions
        region_props = regionprops(labeled)

        good_regions = np.asarray([(r['area'], r['label']) for r in region_props])

        # remove the largest region (the background)
        max_area_index = np.argmax(good_regions[:, 0])
        good_regions = np.delete(good_regions, max_area_index, axis=0)

        # remove all regions smaller than min_area (little holes in the image)
        good_regions = good_regions[good_regions[:, 0] > MIN_AREA]
        label_mask = np.isin(labeled, good_regions[:, 1])
        labeled = np.where(label_mask, labeled, 0)

        # regen the region_props for remaining regions
        region_props = regionprops(labeled)

        # plt.imshow(image)
        # plt.show()

        # convex = morphology.convex_hull_object(labeled)
        # plt.imshow(convex)
        # plt.show()

        # convex_edges = feature.canny(convex)

        # plt.imshow(convex_edges)
        # plt.show()

        # radii = range(5,50,5)
        # hough = transform.hough_circle(convex_edges, radii)
        # img = np.empty_like(image)
        # acc, cx, cy, rad = transform.hough_circle_peaks(hough, radii, min_xdistance=10, min_ydistance=10, threshold=0.85*np.max(hough))
        # for center_y, center_x, radius in zip(cy, cx, rad):
        #     circy, circx = draw.circle_perimeter(center_y, center_x, radius,
        #                                     shape=img.shape)
        #     img[circy, circx] = (255, 255, 255)
        # plt.imshow(img)
        # plt.show()

        # for each region
        for hole_index in range(0, len(np.unique(labeled)) - 1):
            hole = labeled == hole_index
            # check if hole, if it is then append to holes
            hole = self.identify_hole(hole, region_props[hole_index])
            if hole:
                # radius in pixels to diameter in mm
                hole[2] = hole[2] * 2 * MM_PER_PIX
                print(f'diameter estimate: {hole[2]}')
                holes.append(hole)

        # plot the holes, just for visual confirmation
        fig, ax = plt.subplots()
        ax.imshow(image)
        for hole in holes:
            cx, cy, rad, depth = hole
            ax.add_patch(plt.Circle((cx, cy), rad / 2 / MM_PER_PIX))
        plt.show()

        return holes

    def identify_hole(self, hole, regionprop):
        """identify the size and position of a single hole given by an image

        Args:
            hole (image, ndarray (:,:,1) dtype=uint8): a single-channel image with dtype uint8 that only contains pixels for the hole you wish to find

        Returns:
            (x, y, rad): a tup[le containing, respectively: the x followed by the y position in the image of the center of the hole measured in pixels, the radius of the hole in pixels
        """
        # drill_radius_step = 2
        # drill_max_radius = 50
        # drill_sizes = np.arange(drill_radius_step, drill_max_radius, drill_radius_step)
        # res = transform.hough_circle(morphology.convex_hull_image(hole), drill_sizes)
        # accum, cx, cy, rad = transform.hough_circle_peaks(res, drill_sizes, total_num_peaks=1)
        ecc_thresh = 0.4
        radius_estimate = regionprop.equivalent_diameter_area / 2
        if (regionprop.eccentricity < ecc_thresh):
            centroid = regionprop.centroid
            return [centroid[1], centroid[0], radius_estimate, 0]
        else:
            return None

    def show(self):
        pcd = o3d.io.read_point_cloud(self.file_path)
        # todo:: check for unable to open file warning
        o3d.visualization.draw_geometries([pcd])

    def show(self, pcd, win_name):
        o3d.visualization.draw_geometries([pcd], window_name=win_name)
