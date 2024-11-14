import time

import cv2
import numpy as np
import torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor


class SegmentAnythingUI:
    def __init__(self, size='small'):
        self.femur_point = None
        self.tibia_point = None
        self.mask_pointA = None
        self.mask_pointB = None
        self.button_shown = False
        self.button_size = (150, 40)  # Width and height of the button
        self.button_position = None  # To be set based on image size
        self.original_image = None
        self.image = None
        self.mask_generated = False
        self.mask = None

        # SAM2 Setup
        self.device = torch.device("cuda")
        torch.autocast("cuda", dtype=torch.bfloat16).__enter__()

        if size == 'small':
            sam2_checkpoint = (
                "/ros_ws/src/segment-anything-2/checkpoints/sam2_hiera_small.pt"
            )
            model_cfg = "sam2_hiera_s.yaml"
        elif size == 'large':
            sam2_checkpoint = (
                "/ros_ws/src/segment-anything-2/checkpoints/sam2_hiera_large.pt"
            )
            model_cfg = "sam2_hiera_l.yaml"
        else:
            raise ValueError("Invalid size. Choose 'small' or 'large'.")

        t0 = time.time()
        sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=self.device)
        self.predictor = SAM2ImagePredictor(sam2_model)
        print(f"SAM2 model loaded in {time.time() - t0:.2f} seconds.")
        self.warmup()

    def warmup(self):
        random_image = np.random.rand(640, 640, 3).astype(np.uint8)
        self.predictor.set_image(random_image)
        self.predictor.predict(
            point_coords=np.array([[0, 0]]), point_labels=np.array([1]), multimask_output=False
        )
        print("Warmed up SAM2 model.")

    def select_point(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Handle button click if both points are selected and button is shown
            if self.button_shown:
                if (
                    self.button_position[0]
                    <= x
                    <= self.button_position[0] + self.button_size[0]
                    and self.button_position[1]
                    <= y
                    <= self.button_position[1] + self.button_size[1]
                ):
                    pass

            # If femur point is not selected, select it
            if self.femur_point is None:
                self.femur_point = (x, y)
                self.update_image_with_points()
            # If femur is selected but tibia is not, select tibia
            elif self.tibia_point is None:
                self.tibia_point = (x, y)
                self.update_image_with_points()
                self.show_register_button()  # Show the button after tibia is selected

    def update_prompt(self, text):
        self.image = (
            self.original_image.copy()
        )  # Reload the clean image without text or circles
        cv2.putText(
            self.image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2
        )
        self.update_image_with_points()  # Make sure to redraw the points when updating the prompt

    def update_image_with_points(self):
        # Redraw the femur point if selected
        if self.femur_point is not None:
            cv2.circle(
                self.image, self.femur_point, 5, (255, 0, 0), -1
            )  # Draw blue circle

        # Redraw the tibia point if selected
        if self.tibia_point is not None:
            cv2.circle(
                self.image, self.tibia_point, 5, (0, 165, 255), -1
            )  # Draw orange circle

        # Update the displayed image
        cv2.imshow("Image", self.image)

    def update_image_with_mask(self, mask,display=True):
        # Define the RGBA color (blue with 60% opacity)
        color = np.array([30, 144, 255, 153])  # Converted 0.6 to 255 scale for opacity

        # Get height and width from the mask shape
        h, w = mask.shape[:2]

        # Ensure the mask is binary (0 or 1)
        mask = (mask > 0).astype(np.uint8)

        # Create an RGBA mask image by stacking the color array based on the mask
        mask_image = np.zeros((h, w, 4), dtype=np.uint8)
        for i in range(4):  # Iterate through each color channel
            mask_image[:, :, i] = mask * color[i]

        # Ensure the original image is in RGBA format
        if self.image.shape[2] == 3:  # Check if image is RGB
            self.image = cv2.cvtColor(self.image, cv2.COLOR_RGB2RGBA)

        # Combine the original image with the mask image using cv2.addWeighted
        # Note: Both images must have the same number of channels and depth
        self.image = cv2.addWeighted(self.image, 0.5, mask_image, 0.5, 0)

        # Display the updated image
        if display:
            cv2.imshow("Image", self.image)

    def update_image_with_annotations(self, mask, points,display=True):
        # Draw a cv2.minAreaRect around the mask
        contour_mask = mask.astype(np.uint8)
        contours, _ = cv2.findContours(
            contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        largest_contour = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(self.image, [box], 0, (0, 0, 255), 2)

        # Mark the center of the mask
        center = (int(rect[0][0]), int(rect[0][1]))
        cv2.circle(self.image, center, 5, (0, 255, 0), -1)

        # Find the side of the box that is closest to the Tibia point
        tibia_point = np.array(self.tibia_point)
        closest_side_start = None
        closest_side_end = None
        min_distance = float("inf")

        for i in range(4):
            side_start = box[i]
            side_end = box[(i + 1) % 4]
            side_midpoint = (side_start + side_end) / 2
            distance = np.linalg.norm(tibia_point - side_midpoint)

            if distance < min_distance:
                min_distance = distance
                closest_side_start = side_start
                closest_side_end = side_end

        # Draw the closest side
        if closest_side_start is not None and closest_side_end is not None:
            cv2.line(
                self.image,
                tuple(map(int, closest_side_start)),
                tuple(map(int, closest_side_end)),
                (30, 255, 0),
                2,
            )

            # Calculate the midpoint of the closest side
            midpoint = (closest_side_start + closest_side_end) / 2

            # Draw an arrow from the mask center to the midpoint of the closest side
            cv2.arrowedLine(
                self.image,
                center,
                tuple(map(int, midpoint)),
                (144, 30, 255),
                2,
                tipLength=0.1,
            )
            self.mask_pointA = center
            self.mask_pointB = tuple(map(int, midpoint))
        if display:
            cv2.imshow("Image", self.image)

    def show_register_button(self):
        # Reload the image without any text to clear the Tibia prompt
        self.image = self.original_image.copy()
        # Redraw the points
        self.update_image_with_points()

        # Calculate the button position dynamically based on the image size
        self.button_position = (
            self.image.shape[1] - self.button_size[0] - 10,
            self.image.shape[0] - self.button_size[1] - 10,
        )

        # Draw the "Register" button at the calculated position
        self.button_shown = True
        cv2.rectangle(
            self.image,
            self.button_position,
            (
                self.button_position[0] + self.button_size[0],
                self.button_position[1] + self.button_size[1],
            ),
            (0, 255, 0),
            -1,
        )
        cv2.putText(
            self.image,
            "Register",
            (self.button_position[0] + 10, self.button_position[1] + 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
        )
        # Show Text at the top. Esc to reset | Enter to register
        self.update_prompt("Esc to reset | Enter to register")
        cv2.imshow("Image", self.image)

    def generate_mask(self,femur_point,tibia_point,display=True):
        input_point = np.array([femur_point, tibia_point])
        input_label = np.array([1, 0]) # Extracts Femur Mask

        self.predictor.set_image(self.original_image)
        t0 = time.time()
        masks, scores, logits = self.predictor.predict(
            point_coords=input_point, point_labels=input_label, multimask_output=False
        )
        print(f"Mask generation completed in {time.time() - t0:.2f} seconds.")

        mask = masks[0]
        # Remove noise from the mask
        kernel_size = 3
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
        mask = cv2.erode(mask.astype(np.uint8), kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1).astype(bool)
        self.update_image_with_mask(mask,display)
        self.update_image_with_annotations(mask, [femur_point, tibia_point],display)
        self.mask = mask

    def segment_using_ui(self, image):
        self.original_image = image.copy()
        self.update_prompt("Click a point on the Femur")

        # Set mouse callback function
        cv2.setMouseCallback("Image", self.select_point)

        # Main loop to handle key events
        while True:
            key = cv2.waitKey(1) & 0xFF
            # Press 'Esc' to reset the points and clear the image
            if key == 27:  # Esc key
                self.femur_point = None
                self.tibia_point = None
                self.button_shown = False  # Hide the register button
                self.update_prompt("Click a point on the Femur")
                self.mask_generated = False
                print("Points reset.")

            # Update the prompt after the femur is selected
            if self.femur_point is not None and self.tibia_point is None:
                self.update_prompt("Click a point on the Tibia")

            if self.femur_point is not None and self.tibia_point is not None:
                if not self.mask_generated:
                    self.generate_mask(self.femur_point, self.tibia_point)
                    self.mask_generated = True

            # Press 'Enter' to register
            if key == 13 and self.mask_generated:
                break
            
        # Clean up
        cv2.destroyAllWindows()

        # Return Mask + Relevant Points and reset instance variables
        result = (
            self.mask,
            [self.femur_point, self.tibia_point],
            [self.mask_pointA, self.mask_pointB]
        )
        
        # Reset instance variables
        self.reset()
        return result
    
    def segment_using_points(self, image, femur_point, tibia_point):
        self.original_image = image.copy()
        self.femur_point = femur_point
        self.tibia_point = tibia_point
        self.generate_mask(femur_point, tibia_point, display=False)
        result = (
            self.mask,
            [femur_point, tibia_point],
            [self.mask_pointA, self.mask_pointB]
        )
        self.reset()
        return result
    
    def reset(self):
        self.mask = None
        self.femur_point = self.tibia_point = None
        self.mask_pointA = self.mask_pointB = None
        self.mask_generated = False
        self.button_shown = False