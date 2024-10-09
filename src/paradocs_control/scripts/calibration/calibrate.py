import os
import pandas as pd
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
import cv2
# from sklearn.linear_model import LinearRegression


def main():
    ground_truth, ee_transform, camera_point = read_data(4, 3)

    x_offset = 0.000
    y_offset = 0.000

    # print("Ground Truth:", ground_truth.shape)

    # Add offsets to the ground truth points
    ground_truth[:, 0] -= x_offset
    ground_truth[:, 1] += y_offset
    
    camera_extrinsic_matrix_estimate = np.eye(4)
    point = [0.045043, -0.00705, 0.17073] # trial and error


    # point = [0.045043, -0.007943, 0.1763430]
    
    # point = [0.042089, -0.009798, 0.176463]
    # point = [0.043743, -0.011224, 0.178622]
    # point = [0.0042960, -0.009983, 0.178501]
    quaternion = [0.0024341314115742248, -0.006789088277323106, 0.7069977614418309, 0.7071790074661958]
    # quaternion = [-0.706533, -0.002245, -0.707677, 0.000451]
    # quaternion = [0.705156, -0.006922, 0.708884, 0.013812]
    # quaternion = [0.002011, 0.000538, 0.706159, 0.708050]
    rotation = R.from_quat(quaternion)
    rotation_matrix = rotation.as_matrix()
    camera_extrinsic_matrix_estimate[:3, :3] = rotation_matrix
    camera_extrinsic_matrix_estimate[:3, 3] = point


    point_index = 0
    one_value = ee_transform[point_index] @ camera_extrinsic_matrix_estimate @ camera_point[point_index]
    print("transformed value:", one_value)
    print("ground truth:", ground_truth[point_index])
    error = evaluate_error(ground_truth, ee_transform, camera_point, camera_extrinsic_matrix_estimate)
    error = error * 1000
    print("Overall Error:", error)
    print("x_error:", (one_value[0] - ground_truth[point_index][0])*1000)
    print("y_error:", (one_value[1] - ground_truth[point_index][1])*1000)
    print("z_error:", (one_value[2] - ground_truth[point_index][2])*1000)

    
    # output_iterative = solve_matrix_least_squares_iterative(ground_truth, ee_transform, camera_point, camera_extrinsic_matrix_estimate)
    # print("Iterative solution:")
    # print(output_iterative[0])
    # print("Error:", output_iterative[1])
    # print("Iterations:", output_iterative[2])

    # output_direct = solve_matrix_least_squares_direct(ground_truth, ee_transform, camera_point)
    # calculated_matrix = output_direct[0]
    # print("Direct solution:")
    # print(output_direct[0])
    # print("Error:", output_direct[1])

    # output_als = solve_als_with_regularization(ground_truth, ee_transform, camera_point)
    # print("ALS solution:")
    # print(output_als[0])
    # print("Error:", output_als[1])

    # calculated_matrix = last_try(ground_truth, ee_transform, camera_point, camera_extrinsic_matrix_estimate)
    # print("Last try solution:")
    # print(calculated_matrix)

    # calculated_matrix = try_pnp(ground_truth, camera_point, ee_transform)

    # one_value = ee_transform[0] @ calculated_matrix @ camera_point[0]
    # # one_value = one_value / one_value[3]
    # print("curr value:", one_value)
    # error = evaluate_error(ground_truth, ee_transform, camera_point, calculated_matrix)
    # print("curr Error:", error)

    # error = error_function(last_try_soln[:3,:4].flatten(), ee_transform[0], camera_point[0], ground_truth[0])
    # print("Error:", error)

    # one_value = ee_transform[0] @ output_direct[0] @ camera_point[0]
    # one_value = one_value / one_value[3]
    # print("One value:", one_value)
    # error = evaluate_error(ground_truth, ee_transform, camera_point, output_direct[0])
    # print("Error:", error)
    

    # one_value = ee_transform[0] @ output_direct[0] @ camera_point[0]
    # one_value = one_value / one_value[3]
    # print("One value:", one_value)
    # error = evaluate_error(ground_truth, ee_transform, camera_point, output_direct[0])
    # print("Error:", error)


    # one_value = ee_transform[0] @ camera_extrinsic_matrix_estimate @ camera_point[0]
    # print("initial value:", one_value)
    # error = evaluate_error(ground_truth, ee_transform, camera_point, camera_extrinsic_matrix_estimate)
    # print("initial Error:", error)



    # print(ee_transform[0])
    # print(camera_point[0])
    # # one_value = np.linalg.inv(ee_transform[0]) @ ground_truth[0] @ np.linalg.pinv(camera_point[0])
    # one_value = np.linalg.inv( camera_point[0]  @ np.linalg.inv(np.linalg.inv(ee_transform[0]) @ ground_truth[0] ) )
    # print("One value:", one_value)

def try_pnp(ground_truth, camera_point, ee_transform):

    ee_transform = np.array(ee_transform)
    object_points = []
    for i in range(90):
        point = np.linalg.inv(ee_transform[i]) @ ground_truth[i]
        object_points.append(point)
        # print(point)
        # exit()
    object_points = np.array(object_points)
    object_points = object_points[:, :3]  # Remove the homogeneous coordinate
    image_points = camera_point[:, :2]  # Remove the homogeneous coordinate

    # Camera matrix (assuming fx = fy = 1 and cx = cy = 0 for simplicity)
    camera_matrix = np.eye(3)

    # Distortion coefficients (assuming no distortion)
    dist_coeffs = np.zeros(4)

    print(f"ground_truth shape: {object_points.shape}, dtype: {ground_truth.dtype}")
    print(f"camera_point shape: {image_points.shape}, dtype: {camera_point.dtype}")
    print(f"ee_transform shape: {ee_transform.shape}, dtype: {ee_transform.dtype}")

    if ground_truth.shape[0] < 4 or camera_point.shape[0] < 4:
        raise ValueError("Not enough points for solvePnPRansac. At least 4 points are required.")

    # Use solvePnPRansac to estimate the pose
    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points, image_points, camera_matrix, dist_coeffs
    )

    if success:
        # Convert rotation vector to rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        # Construct the extrinsic matrix
        camera_extrinsic_matrix = np.eye(4)
        camera_extrinsic_matrix[:3, :3] = rotation_matrix
        camera_extrinsic_matrix[:3, 3] = tvec.flatten()

        print("Estimated camera extrinsic matrix using RANSAC PnP:")
        print(camera_extrinsic_matrix)
        return camera_extrinsic_matrix
    else:
        print("solvePnPRansac failed to find a solution.")


def solve_matrix_least_squares_iterative(A_list, B_list, C_list, X_init, max_iterations=5000, tolerance=1e-20):
    n = len(A_list)
    
    X = X_init.copy()
    prev_error = float('inf')
    
    for iteration in range(max_iterations):
        # Prepare the system
        Y = np.vstack(A_list)  # Stack all A matrices vertically
        Phi = np.zeros((n*4, 16))  # 16 because X is 4x4 flattened
        
        for i in range(n):
            Phi[i*4:(i+1)*4, :] = np.kron(C_list[i], B_list[i])
        
        # Solve using least squares
        vec_X = np.linalg.lstsq(Phi, Y.flatten(), rcond=None)[0]
        
        # Reshape the solution
        X_new = vec_X.reshape(4, 4)
        
        # Calculate error
        error = np.mean([np.sum((A - B @ X_new @ C.T)**2) for A, B, C in zip(A_list, B_list, C_list)])
        
        # Check for convergence
        if abs(error - prev_error) < tolerance:
            break
        
        X = X_new
        prev_error = error
        print(f"Iteration: {iteration}, Error: {error}")
    
    return X, error, iteration + 1


# without initial estimate
def solve_matrix_least_squares_direct(A_list, B_list, C_list):
    n = len(A_list)
    
    # Prepare the system
    Y = np.vstack(A_list).flatten()  # Stack all A matrices vertically and flatten
    Phi = np.zeros((n*4, 16))  # 16 because X is 4x4 flattened
    
    for i in range(n):
        Phi[i*4:(i+1)*4, :] = np.kron(B_list[i], C_list[i].T)
    
    # Solve using least squares
    vec_X = np.linalg.lstsq(Phi, Y, rcond=None)[0]
    
    # Reshape the solution
    X = vec_X.reshape(4, 4)
    
    # Calculate error
    error = np.mean([np.sum((A - B @ X @ C.T)**2) for A, B, C in zip(A_list, B_list, C_list)])
    
    return X, error

# Read the data from the CSV files, and store them in the variables
def read_data(gt_points, orientations):
    ground_truth = []
    ee_transform = []
    camera_point = []
    # Define the paths to the files
    # base_dir = '/home/paradocs/paradocs_docker_ws/src/tekkneeca/src/paradocs_control/scripts/calibration_error_eval'
    # base_dir = 'calibration_error_eval'
    base_dir = ''
    csv_file_path = os.path.join(base_dir, 'calibration_data.csv')
    transforms_file_path = os.path.join(base_dir, 'calibration_transforms.csv')

    calibration_data = None
    calibration_transforms = None

    # Read the CSV file using pandas with ";" as the delimiter
    try:
        calibration_data = np.array(pd.read_csv(csv_file_path, delimiter=';'))
        # print(calibration_data.shape)
        # print(calibration_data)
        # print("Calibration Data:")
        # print(calibration_data
    except FileNotFoundError:
        print(f"File not found: {csv_file_path}")

    # Read the transforms file using pandas with ";" as the delimiter
    try:
        calibration_transforms = np.array( pd.read_csv(transforms_file_path, delimiter=';', converters={i: eval for i in range(8)}).values)
        # print("\nCalibration Transforms:")
        # print(calibration_transforms)
    except FileNotFoundError:
        print(f"File not found: {transforms_file_path}")


    transform_index = -1
    for i in range(0,gt_points*orientations):

        point_str = calibration_data[i][0]
        point_float_array = np.array([float(coord) for coord in point_str.strip('[]').split(',')])
        # ground_truth.append(calibration_data[i][0])
        ground_truth.append(point_float_array)
        
        # Convert the string representation of the point to a float array
        point_str = calibration_data[i][2]
        point_float_array = np.array([float(coord) for coord in point_str.strip('[]').split(',')])
        
        # camera_point.append(np.append(point_float_array, 1))
        camera_point.append(point_float_array)

        if i%(gt_points+1) == 0:
            transform_index = transform_index + 1

        # transform = np.array([float(x) for x in calibration_transforms[transform_index][0].strip('[').strip(']').strip(' ').split(',')])
        # transform = calibration_transforms[transform_index][0]
        # print(transform)
        position= calibration_transforms[transform_index][0][0]
        quaternion= calibration_transforms[transform_index][0][1]
        
        # print(quaternion)



        # Convert quaternion to rotation matrix
        # Convert quaternion to rotation matrix using scipy
        rotation = R.from_quat(quaternion)
        rotation_matrix = rotation.as_matrix()

        # Create the transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = position

        ee_transform.append(transformation_matrix)

    ground_truth = np.array(ground_truth)   
    camera_point = np.array(camera_point)

    ground_truth = np.hstack((ground_truth, np.ones((ground_truth.shape[0], 1))))
    camera_point = np.hstack((camera_point, np.ones((camera_point.shape[0], 1))))
    
    return ground_truth, ee_transform, camera_point

def evaluate_error(ground_truth, ee_transform, camera_point, camera_extrinsic_matrix):
    error = 0
    for i in range(len(ground_truth)):
        error += np.sum((ground_truth[i] - ee_transform[i] @ camera_extrinsic_matrix @ camera_point[i])**2)

        # ground_truth = ee_transform[i] @ camera_extrinsic_matrix @ camera_point[i]
        # A .C-1 = B X C . C-1

        # A = K X
        # K  = kronecker product


    error /= len(ground_truth)
    return error

def error_function(camera_extrinsic_vector, ee_transform, camera_point, ground_truth):
    # Reshape the vector back into a 4x4 matrix (with the last row fixed)
    camera_extrinsic_matrix = np.vstack([camera_extrinsic_vector.reshape(3, 4), [0, 0, 0, 1]])
    error = []
    
    for i in range(len(ground_truth)):
        # Compute the predicted ground truth for each point
        predicted = ee_transform[i] @ camera_extrinsic_matrix @ camera_point[i].T
        # Calculate the error (difference)
        temp_error = np.subtract(ground_truth[i], predicted)
        temp_error = temp_error * 1000
        # temp_error[0] = temp_error[0] * 10 
        # temp_error[1] = temp_error[1] * 10
        # temp_error[2] = temp_error[2] **2
        error.append(temp_error.flatten())
    
    # Return the flattened error for optimization
    return np.concatenate(error)

def plot( ee_transform, camera_point, ground_truth, result):
    import matplotlib.pyplot as plt

    

    # Plot the results
    predicted_points = []
    for i in range(len(ground_truth)):
        predicted = ee_transform[i] @ result @ camera_point[i].T
        predicted_points.append(predicted[:3])  # Only take the x, y, z coordinates

    predicted_points = np.array(predicted_points)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot ground truth points
    ax.scatter(ground_truth[:, 0], ground_truth[:, 1], ground_truth[:, 2], c='r', marker='o', label='Ground Truth')

    # Plot predicted points
    ax.scatter(predicted_points[:, 0], predicted_points[:, 1], predicted_points[:, 2], c='b', marker='^', label='Predicted')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.legend()

    plt.show()

def last_try(ground_truth, ee_transform, camera_point, initial_guess):
    plot(ee_transform, camera_point, ground_truth, initial_guess)

    # initial_guess = np.eye(4).flatten()  # Start with an identity matrix for the extrinsic matrix
    initial_guess = initial_guess[:3, :4].flatten() 

    result = least_squares(error_function, initial_guess, args=(ee_transform, camera_point, ground_truth))
    
    

    

    # Reconstruct the full 4x4 camera extrinsic matrix with the last row fixed as [0, 0, 0, 1]
    camera_extrinsic_matrix_optimal = np.vstack([result.x.reshape(3, 4), [0, 0, 0, 1]])
    plot(ee_transform, camera_point, ground_truth, camera_extrinsic_matrix_optimal)
    #plot with initial guess
    
    return camera_extrinsic_matrix_optimal

    

def solve_als_with_regularization(A_list, B_list, C_list, max_iterations=100, tolerance=1e-6, lambda_reg=0.1):
    n = len(A_list)
    X = np.random.rand(4, 4)  # Random initial guess
    
    for iteration in range(max_iterations):
        X_prev = X.copy()
        
        # Solve for X
        B_combined = np.vstack([B @ np.kron(C, np.eye(4)) for B, C in zip(B_list, C_list)])
        A_combined = np.vstack(A_list).flatten()
        
        # Add regularization
        B_combined_reg = np.vstack([B_combined, np.sqrt(lambda_reg) * np.eye(16)])
        A_combined_reg = np.concatenate([A_combined, np.zeros(16)])
        
        X = np.linalg.lstsq(B_combined_reg, A_combined_reg, rcond=None)[0].reshape(4, 4)
        
        # Check for convergence
        if np.linalg.norm(X - X_prev) < tolerance:
            break
    
    # Calculate final error
    error = np.mean([np.sum((A - B @ X @ C.T)**2) for A, B, C in zip(A_list, B_list, C_list)])
    
    return X, error, iteration + 1

if __name__ == "__main__":
    main()