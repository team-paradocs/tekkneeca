import cv2
import numpy as np
import open3d as o3d
import time

class RegistrationPipeline:
    def __init__(self):
        # D405 Intrinsics
        self.fx = 425.19189453125
        self.fy = 424.6562805175781
        self.cx = 422.978515625
        self.cy = 242.1155242919922


    def register(self, mask, source_cloud, raw_cloud, annotated_points, mask_points):
        mask_cloud = self.unproject_mask(mask, raw_cloud.points)
        target_cloud = mask_cloud.voxel_down_sample(voxel_size=0.003)

        # print(f"Number of points in source cloud: {len(self.source_cloud.points)}")
        # print(f"Number of points in target cloud: {len(target_cloud.points)}")

        init_transformation = self.global_registration(source_cloud, target_cloud, annotated_points, mask_points)
        transform, fitness = self.directional_icp(source_cloud, target_cloud, init_transformation)
        return transform, fitness
        

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
        # Mask Points are 2 points (x,y,z) of a vector that is the direction of the femur
        # Get Yaw from the vector if provided else default to 180
        yaw_deg = 180
        pitch_deg = 0
        if mask_points is not None:
            yaw_deg = -np.degrees(np.arctan2(mask_points[1][1] - mask_points[0][1], mask_points[1][0] - mask_points[0][0]))
            yaw_deg += 90
        roll, pitch, yaw = np.radians([180, pitch_deg, yaw_deg])  # Convert degrees to radians
        rotation = o3d.geometry.get_rotation_matrix_from_xyz((roll, pitch, yaw))
        rotation_4x4 = np.eye(4)  # Expand to 4x4 matrix
        rotation_4x4[0:3, 0:3] = rotation  # Set the top-left 3x3 to the rotation matrix

        px, py, pz = annotated_points[0]
        X = (px - self.cx) * pz / self.fx
        Y = (py - self.cy) * pz / self.fy
        translation = np.eye(4)
        translation[0:3, 3] = np.array([X,Y,pz])

        # Combine transformations
        transformation = translation @ rotation_4x4 @ translation_to_origin

        return transformation
    
    def icp(self, source, target, initial_transformation):
        threshold = 0.01
        max_iter = 50
        source.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        target.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        loss = o3d.pipelines.registration.TukeyLoss(k=0.1)
        p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)

        reg_result = o3d.pipelines.registration.registration_icp(
            source, target, threshold, initial_transformation,
            p2l,
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
        )
        print(f"ICP Fitness: {reg_result.fitness}")
        return reg_result.transformation

    def multi_icp(self, source, target, initial_transformation, max_trials=5):
        min_fitness = 0.9
        max_iter = 50
        threshold = 0.01
        source.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        target.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        loss = o3d.pipelines.registration.TukeyLoss(k=0.1)
        p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)

        while max_trials > 0:
            reg_result = o3d.pipelines.registration.registration_icp(
                source, target, threshold, initial_transformation,
                p2l,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
            )
            if reg_result.fitness > min_fitness:
                print(f"ICP Fitness: {reg_result.fitness}")
                return reg_result.transformation
            max_trials -= 1
        print(f"Failed to converge. Final Fitness: {reg_result.fitness}")
        return reg_result.transformation

    def directional_icp(self, source, target, initial_transformation, offset=0.02):
        # Define ICP parameters
        min_fitness = 0.9
        max_iter = 50
        threshold = 0.01
        source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        loss = o3d.pipelines.registration.TukeyLoss(k=0.1)
        p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
        
        # Define directional offsets
        offsets = [
            [ offset,  0.0,    0.0],  # +x
            [-offset,  0.0,    0.0],  # -x
            [ 0.0,     offset, 0.0],  # +y
            [ 0.0,    -offset, 0.0],  # -y
            [ 0.0,     0.0,    offset], # +z
            [ 0.0,     0.0,   -offset], # -z
            [ offset,  offset, 0.0],    # +x+y
            [ offset,  0.0,    offset], # +x+z
            [ 0.0,     offset, offset]  # +y+z
        ]
        
        best_fitness = 0
        best_transformation = initial_transformation

        # Perform ICP trials with each directional offset
        for i, direction in enumerate(offsets):
            # Create a new transformation matrix with the current offset
            directional_transform = np.copy(initial_transformation)  # Use np.copy instead of np.array
            directional_transform[:3, 3] += np.array(direction)  # Convert direction to numpy array
            
            # Run ICP with the current transformation
            reg_result = o3d.pipelines.registration.registration_icp(
                source, target, threshold, directional_transform,
                p2l,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
            )
            
            # Check if the fitness is the best found so far
            if reg_result.fitness > best_fitness:
                best_fitness = reg_result.fitness
                best_transformation = reg_result.transformation
            
            # print(f"Trial {i+1} - Direction: {direction} - Fitness: {reg_result.fitness}")
            
            # Early exit if fitness meets the threshold
            # if reg_result.fitness > min_fitness:
            #     print(f"Early convergence reached with fitness: {reg_result.fitness}")
            #     return reg_result.transformation
        
        # Final output with best fitness result
        print(f"Best fitness achieved: {best_fitness}")
        return best_transformation, best_fitness

    def ransac_icp(self, source, target, initial_transformation, trials=300, until_convergence=False):
        '''
        RANSAC ICP
        '''
        threshold = 0.01
        max_iter = 50
        best_transformation = initial_transformation
        best_fitness = 0.0
        min_fitness = 0.90

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

        if until_convergence:
            # If still less than min fitness, start a while loop to try to converge
            extra_trials = 0
            while best_fitness < min_fitness:
                noise = np.random.normal(0, 0.02, (4, 4))
                noisy_transformation = best_transformation + noise
                reg_result = o3d.pipelines.registration.registration_icp(
                    source, target, threshold, noisy_transformation,
                    p2l,
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
                )

                if reg_result.fitness > best_fitness:
                    best_fitness = reg_result.fitness
                    best_transformation = reg_result.transformation

                extra_trials += 1

        print(f"Best Fitness: {best_fitness}")
        if extra_trials > 0:
            print(f"Extra trials: {extra_trials}")
        return best_transformation, best_fitness
