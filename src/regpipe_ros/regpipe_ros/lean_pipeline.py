import cv2
import numpy as np
import open3d as o3d
import time

class LeanPipeline:
    def __init__(self):
        self.fx = 425.19189453125
        self.fy = 424.6562805175781
        self.cx = 422.978515625
        self.cy = 242.1155242919922

        self.intrinsics = np.array([
            [self.fx, 0,  self.cx],
            [0,  self.fy, self.cy],
            [0,  0,  1]
        ])

    def unproject_mask(self, mask, points):
        mask = np.fliplr(mask)
        points = np.asarray(points)
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        u = mask.shape[1] - 1 - ((x * self.fx / z) + self.cx).astype(int)
        v = ((y * self.fy / z) + self.cy).astype(int)

        valid = (u >= 0) & (u < mask.shape[1]) & (v >= 0) & (v < mask.shape[0])

        u_valid = u[valid]
        v_valid = v[valid]
        points_valid = points[valid]

        mask_values = mask[v_valid, u_valid]
        final_points = points_valid[mask_values]

        mask_pcd = o3d.geometry.PointCloud()
        mask_pcd.points = o3d.utility.Vector3dVector(final_points)

        return mask_pcd

    def filter_pcd(self,pcd,nb_neighbors=20,radius=0.01):
        # Use radius outlier removal
        cl, ind = pcd.remove_radius_outlier(nb_points=nb_neighbors, radius=radius)
        return pcd.select_by_index(ind)

    def global_registration(self, source, target, annotated_points, mask_points):
        '''
        Centroid-based transformation estimation
        '''
        source_center = source.get_center()
        target_center = target.get_center()

        # Translate source to origin
        translation_to_origin = np.eye(4)
        translation_to_origin[0:3, 3] = -source_center

        # Create a rotation matrix
        # Mask Points are 2 points (x,y) of a vector that is the direction of the femur
        # It is a list of 2 int tuples (x,y)
        # Get Yaw from the vector if provided else default to 180
        yaw_deg = 180
        if mask_points is not None:
            yaw_deg = np.degrees(np.arctan2(mask_points[1][1] - mask_points[0][1], mask_points[1][0] - mask_points[0][0]))
            yaw_deg += 90
            print(f"Yaw: {yaw_deg}")
        roll, pitch, yaw = np.radians([180, 0, yaw_deg])  # Convert degrees to radians
        rotation = o3d.geometry.get_rotation_matrix_from_xyz((roll, pitch, yaw))
        rotation_4x4 = np.eye(4)  # Expand to 4x4 matrix
        rotation_4x4[0:3, 0:3] = rotation  # Set the top-left 3x3 to the rotation matrix

        # Translate back to target's position
        translation_back = np.eye(4)
        translation_back[0:3, 3] = target_center

        # Get Distance of Femur Point to Target Center
        px, py = annotated_points[0]
        fx, fy = self.intrinsics[0,0], self.intrinsics[1,1]
        cx, cy = self.intrinsics[0,2], self.intrinsics[1,2]
        z = 0.404 # Temporary hardcoding of depth
        X = (px - cx) *z / fx
        Y = (py - cy) *z / fy
        print(f"Femur Point X: {X}, Y: {Y}")
        # Get distance vector to target center
        distance_vector = target_center - np.array([X,Y,0])
        print(f"Distance Vector: {distance_vector}")
        translation_back[0:3, 3] = np.array([X,Y,target_center[2]])

        # Combine transformations
        transformation = translation_back @ rotation_4x4 @ translation_to_origin

        return transformation

    def ransac_icp(self, source, target, initial_transformation, trials=300):
        '''
        RANSAC ICP
        '''
        threshold = 0.01
        max_iter = 50
        best_transformation = None
        best_fitness = 0.0

        # Compute normals for the source and target point clouds
        source.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        target.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        loss = o3d.pipelines.registration.TukeyLoss(k=0.1)
        p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)

        # RANSAC Trials
        for i in range(trials):
            # Add Gaussian noise to the initial transformation
            noise = np.random.normal(0, 0.02, (4, 4))
            noisy_transformation = initial_transformation + noise

            # Perform ICP registration
            reg_result = o3d.pipelines.registration.registration_icp(
                source, target, threshold, noisy_transformation,
                p2l,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
            )

            # Update the best transformation based on fitness
            if reg_result.fitness > best_fitness:
                best_fitness = reg_result.fitness
                best_transformation = reg_result.transformation

        print(f"Best Fitness: {best_fitness}")
        return best_transformation
