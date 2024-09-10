import numpy as np
import open3d as o3d

FMR_ENABLED = False
PROBREG = False

if FMR_ENABLED:
    import torch
    from fmr.model import PointNet, Decoder, SolveRegistration
    import fmr.se_math.transforms as transforms

if PROBREG:
    import transforms3d as t3d
    from probreg import filterreg


class Estimator:
    def __init__(self, method='centroid'):
        self.method = method


    def estimate(self, source, target):
        if self.method == 'centroid':
            return self.centroid(source, target)
        elif self.method == 'fpfh':
            return self.fpfh(source, target)
        elif self.method == 'fast_fpfh':
            return self.fast_fpfh(source, target)
        elif self.method == 'fmr':
            if not FMR_ENABLED:
                raise RuntimeError("FMR is not enabled.")
            return self.feature_metric_reg(source, target)
        else:
            print("Invalid estimation method.")
            return None
        

    def centroid(self, source, target):
        '''
        Centroid-based transformation estimation
        '''
        source_center = source.get_center()
        target_center = target.get_center()

        # Translate source to origin
        translation_to_origin = np.eye(4)
        translation_to_origin[0:3, 3] = -source_center

        # Create a rotation matrix
        roll, pitch, yaw = np.radians([180, 0, 90])  # Convert degrees to radians
        rotation = o3d.geometry.get_rotation_matrix_from_xyz((roll, pitch, yaw))
        rotation_4x4 = np.eye(4)  # Expand to 4x4 matrix
        rotation_4x4[0:3, 0:3] = rotation  # Set the top-left 3x3 to the rotation matrix

        # Translate back to target's position
        translation_back = np.eye(4)
        translation_back[0:3, 3] = target_center

        # Combine transformations
        transformation = translation_back @ rotation_4x4 @ translation_to_origin

        return transformation
    
    def fpfh(self, source, target):
        '''
        FPFH-based global registration using RANSAC
        '''
        voxel_size = 0.05
        radius = voxel_size * 2
        distance_threshold = voxel_size * 1.5

        source.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
        target.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
        
        source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            source, o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=100))
        target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            target, o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=100))
        
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source, target, source_fpfh, target_fpfh, True, distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
            [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
             o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
        
        return result.transformation
    

    def fast_fpfh(self,source,target):
        '''
        FPFH-based fast global registration
        '''
        voxel_size = 0.05
        radius = voxel_size * 2
        distance_threshold = voxel_size * 0.5

        source.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
        target.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
        
        source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            source, o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=100))
        target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            target, o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=100))
        
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source, target, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
        
        return result.transformation
    
    def feature_metric_reg(self, source, target):
        '''
        Feature Metric Registration
        Refer - https://github.com/XiaoshuiHuang/fmr
        '''

        dim_k = 1024
        max_iter = 10
        loss_type = 1
        device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')


        source = np.asarray(source.points)
        source = np.expand_dims(source, axis=0)
        target = np.asarray(target.points)
        target = np.expand_dims(target, axis=0)


        ptnet = PointNet(dim_k)
        decoder = Decoder()
        fmr_solver = SolveRegistration(ptnet, decoder, isTest=True)

        model_path = 'fmr/fmr_model_7scene.pth'
        fmr_solver.load_state_dict(torch.load(model_path, map_location=device))
        fmr_solver.to(device)

        fmr_solver.eval()
        with torch.no_grad():
            source = torch.tensor(source, dtype=torch.float).to(device)
            target = torch.tensor(target, dtype=torch.float).to(device)
            fmr_solver.estimate_t(source, target, max_iter)
            print("FMR Model Loaded")


            est_g = fmr_solver.g
            g_hat = est_g.cpu().contiguous().view(4, 4)



        return g_hat.numpy()





class Refiner:
    def __init__(self, method='p2pl_icp'):
        self.method = method


    
    def refine(self, source, target, transformation=np.eye(4)):

        if self.method == 'p2pl_icp':
            return self.p2pl_icp(source, target, transformation)
        elif self.method == 'robust_p2pl_icp':
            return self.robust_p2pl_icp(source, target, transformation)
        elif self.method == 'ransac_icp':
            return self.ransac_icp(source, target, transformation)
        elif self.method == 'filterreg':
            if not PROBREG:
                raise RuntimeError("Probreg is not enabled.")
            return self.ransac_filterreg(source, target, transformation)
        else:
            print("Invalid refinement method.")
            return None
        


    def p2pl_icp(self, source, target, transformation):
        '''
        Open3D Point-to-Plane ICP
        '''
        threshold = 0.02
        max_iter = 2000


        # Compute normals for source and target point clouds
        source.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        target.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))


        reg_result = o3d.pipelines.registration.registration_icp(
            source, target, threshold, transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
        )

        print(reg_result)

        return reg_result.transformation
    
    def robust_p2pl_icp(self, source, target, transformation):
        '''
        Open3D Point-to-Plane ICP with Robust Kernels
        '''

        threshold = 0.02
        max_iter = 2000

        # Compute normals for source and target point clouds
        source.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        target.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        loss = o3d.pipelines.registration.TukeyLoss(k=0.1)
        p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)


        reg_result = o3d.pipelines.registration.registration_icp(
            source, target, threshold, transformation,
            p2l,
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
        )

        print(reg_result)

        return reg_result.transformation
    

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
    
    def filterreg(self, source, target):
        '''
        FilterReg from Probreg
        '''
        objective_type = 'pt2pt'
        tf_param, _, _ = filterreg.registration_filterreg(source, target,
                                                  objective_type=objective_type,
                                                  sigma2=None,
                                                  update_sigma2=True)
        
        rot = tf_param.rot
        t = tf_param.t

        transformation = np.eye(4)
        transformation[0:3, 0:3] = rot
        transformation[0:3, 3] = t

        return transformation
    

    def ransac_filterreg(self, source, target, initial_transformation, trials=10):
        '''
        RANSAC FilterReg
        '''
        best_transformation = None
        best_sig = 1e7
        source.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        target.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        

        for i in range(trials):
            # Add Gaussian noise to the initial transformation
            noise = np.random.normal(0, 0.02, (4, 4))
            noisy_transformation = initial_transformation + noise

            # Perform FilterReg registration
            transformation, sig2, qval = filterreg.registration_filterreg(source, target,
                                                                           objective_type='pt2pt',
                                                                           sigma2=None,update_sigma2=True)
            
            print(f"Trial {i+1} - Sigma2: {sig2}, Qval: {qval}")
            if sig2 < best_sig:
                best_sig = sig2
                best_transformation = transformation

        return best_transformation
    



    

    