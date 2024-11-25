import time

import cv2
import numpy as np
import torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor


class SegmentAnythingUI:
    def __init__(self, size="small"):
        self.bones = ["femur", "tibia"]
        self.label_map = {"femur": np.array([1, 0]), "tibia": np.array([0, 1])}
        self.mask_colors = np.array([[30, 144, 255, 153], [255, 144, 30, 153]])

        self.femur_point = None
        self.tibia_point = None
        self.masks = []
        self.mask_points = []

        self.original_image = None
        self.image = None
        self.mask_generated = False

        # SAM2 Setup
        self.device = torch.device("cuda")
        torch.autocast("cuda", dtype=torch.bfloat16).__enter__()

        if size == "small":
            sam2_checkpoint = (
                "/ros_ws/src/segment-anything-2/checkpoints/sam2_hiera_small.pt"
            )
            model_cfg = "sam2_hiera_s.yaml"
        elif size == "large":
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
            point_coords=np.array([[0, 0]]),
            point_labels=np.array([1]),
            multimask_output=False,
        )
        print("Warmed up SAM2 model.")

    def select_point(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.femur_point is None:
                self.femur_point = (x, y)
            elif self.tibia_point is None:
                self.tibia_point = (x, y)
            self.update_image_with_points()

    def update_image_with_points(self):
        if self.femur_point is not None:
            cv2.circle(self.image, self.femur_point, 5, (255, 0, 0), -1)
        if self.tibia_point is not None:
            cv2.circle(self.image, self.tibia_point, 5, (0, 165, 255), -1)
        cv2.imshow("Image", self.image)

    def update_prompt(self, text):
        self.image = self.original_image.copy()
        cv2.putText(
            self.image,
            text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        self.update_image_with_points()

    def update_image_with_masks(self):
        if self.image.shape[2] == 3:
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGBA)

        for i, mask in enumerate(self.masks):
            color = self.mask_colors[i]
            h, w = mask.shape[:2]
            mask = (mask > 0).astype(np.uint8)
            mask_image = np.zeros((h, w, 4), dtype=np.uint8)
            for j in range(4):
                mask_image[:, :, j] = mask * color[j]
            self.image = cv2.addWeighted(self.image, 0.5, mask_image, 0.5, 0)
        cv2.imshow("Image", self.image)

    def compute_mask_points(self):
        """
        Computes the mask points (center and midpoint of closest side)
        for all masks and stores them in self.mask_points.
        """
        self.mask_points = []  # Clear any existing points

        for i, bone in enumerate(self.bones):
            mask = self.masks[i]

            contour_mask = mask.astype(np.uint8)
            contour_mask = mask.astype(np.uint8)
            contours, _ = cv2.findContours(
                contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            largest_contour = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(largest_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Compute the center of the mask
            center = (int(rect[0][0]), int(rect[0][1]))

            # Opposite point
            if bone == "femur":
                opposite_point = self.tibia_point
            else:
                opposite_point = self.femur_point

            # Find the side of the box closest to the opposite point
            closest_side_start = None
            closest_side_end = None
            min_distance = float("inf")

            for i in range(4):
                side_start = box[i]
                side_end = box[(i + 1) % 4]
                side_midpoint = (side_start + side_end) / 2
                distance = np.linalg.norm(opposite_point - side_midpoint)

                if distance < min_distance:
                    min_distance = distance
                    closest_side_start = side_start
                    closest_side_end = side_end

            if closest_side_start is not None and closest_side_end is not None:
                midpoint = (closest_side_start + closest_side_end) / 2
                self.mask_points.append((center, tuple(map(int, midpoint))))

    def annotate_image(self):
        """
        Annotates the image with mask-related annotations based on self.mask_points.
        """
        for mask, (center, midpoint) in zip(self.masks, self.mask_points):
            contour_mask = mask.astype(np.uint8)
            contours, _ = cv2.findContours(
                contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            largest_contour = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(largest_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Draw the bounding box
            cv2.drawContours(self.image, [box], 0, (0, 0, 255), 2)

            # Draw the mask center
            cv2.circle(self.image, center, 5, (0, 255, 0), -1)

            # Draw the closest side
            closest_side_start = box[0]  # Initialize with dummy values
            closest_side_end = box[1]

            for i in range(4):
                side_start = box[i]
                side_end = box[(i + 1) % 4]

                if (tuple(map(int, side_start)) == midpoint or tuple(map(int, side_end)) == midpoint):
                    closest_side_start = side_start
                    closest_side_end = side_end

            cv2.line(
                self.image,
                tuple(map(int, closest_side_start)),
                tuple(map(int, closest_side_end)),
                (30, 255, 0),
                2,
            )

            # Draw an arrow from the mask center to the midpoint
            cv2.arrowedLine(
                self.image,
                center,
                midpoint,
                (144, 30, 255),
                2,
                tipLength=0.1,
        )
        cv2.imshow("Image", self.image)

    def generate_mask(self, display=True):
        input_point = np.array([self.femur_point, self.tibia_point])

        for i, bone in enumerate(self.bones):
            input_label = self.label_map[bone]
            
            self.predictor.set_image(self.original_image)
            t0 = time.time()
            masks, scores, logits = self.predictor.predict(
                point_coords=input_point, point_labels=input_label, multimask_output=False
            )
            print(f"Mask [{bone}] generation completed in {time.time() - t0:.2f} seconds.")
            mask = masks[0]

            # Remove noise from the mask
            kernel_size = 3
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
            mask = cv2.erode(mask.astype(np.uint8), kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=1).astype(bool)
            self.masks.append(mask)
        self.compute_mask_points()

        if display:
            self.update_image_with_masks()
            self.annotate_image() # Draws arrows

    def segment_using_ui(self, image, bones=None):
        self.original_image = image.copy()
        if bones is not None:
            self.bones = bones

        self.update_prompt("Click a point on the Femur")

        # Set mouse callback function
        cv2.setMouseCallback("Image", self.select_point)

        # Main loop to handle key events
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # Esc key
                self.reset()
                self.update_prompt("Click a point on the Femur")

            # Update the prompt after the femur is selected
            if self.femur_point is not None and self.tibia_point is None:
                self.update_prompt("Click a point on the Tibia")

            if self.femur_point is not None and self.tibia_point is not None:
                if not self.mask_generated:
                    self.update_prompt("Esc to Reset | Enter to Register")
                    self.generate_mask(display=True)
                    self.mask_generated = True

            if key == 13 and self.mask_generated:
                break

        # Clean up
        cv2.destroyAllWindows()

        result = (self.masks, [self.femur_point, self.tibia_point], self.mask_points)
        self.reset()
        return result
    
    def segment_using_points(self, image, femur_point, tibia_point, bones=None):
        self.original_image = image.copy()
        if bones is not None:
            self.bones = bones
        self.femur_point = femur_point
        self.tibia_point = tibia_point
        self.generate_mask(display=True)
        self.mask_generated = True
        result = (self.masks, [femur_point, tibia_point], self.mask_points)
        self.reset()
        cv2.destroyAllWindows()
        return result

    def reset(self):
        self.masks = []
        self.mask_points = []
        self.femur_point = None
        self.tibia_point = None
        self.mask_generated = False
