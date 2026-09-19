import cv2
import numpy as np


class DetectTarget:
    """
    Görüntüdeki en büyük kırmızı bağlı bölgeyi hedef alanı olarak tespit eder.

    Görüntü HSV renk uzayına dönüştürülür ve iki farklı HSV aralığı
    kullanılarak kırmızı pikseller belirlenir. Daha sonra birbirine bağlı
    kırmızı piksel grupları bulunur ve belirlenen minimum alanın üzerindeki
    en büyük bağlı bölge hedef alanı olarak kabul edilir.
    """

    def __init__(self, min_area=1000):
        self.lower_red1 = np.array([0, 100, 80])
        self.upper_red1 = np.array([10, 255, 255])

        self.lower_red2 = np.array([170, 100, 80])
        self.upper_red2 = np.array([180, 255, 255])

        self.min_area = min_area

    def detect_target(self, frame):
        """
        Verilen görüntüdeki en büyük kırmızı bağlı bölgeyi tespit eder.

        Args:
            frame (numpy.ndarray): OpenCV formatında BGR görüntü.

        Returns:
            tuple:
                h_mask (numpy.ndarray):
                    En büyük kırmızı bağlı bölgeyi içeren binary maske.
                    Geçerli bir hedef bulunamazsa tamamen siyah bir maske döner.

                center (tuple):
                    Hedef bölgesinin merkez koordinatı (cx, cy).
                    Geçerli bir hedef bulunamazsa (None, None) döner.
        """

        # BGR -> HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Kırmızı pikselleri bul
        mask1 = cv2.inRange(
            hsv,
            self.lower_red1,
            self.upper_red1
        )

        mask2 = cv2.inRange(
            hsv,
            self.lower_red2,
            self.upper_red2
        )

        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        num_labels, labels, stats, centroids = \
            cv2.connectedComponentsWithStats(
                mask,
                connectivity=8
            )

        # En büyük geçerli bölgeyi bul
        largest_label = 0
        largest_area = 0

        for i in range(1, num_labels):

            area = stats[i, cv2.CC_STAT_AREA]
            
            # Minimum alan filtresi
            if area < self.min_area:
                continue

            if area > largest_area:
                largest_area = area
                largest_label = i
        #print("Largest area:", largest_area)

        # H maskesi
        h_mask = np.zeros_like(mask)

        if largest_label != 0:
            h_mask[labels == largest_label] = 255
            cx, cy = centroids[largest_label]
        else:
            cx, cy = None, None
        
        # Test kodları, gerçek kamera test edin
        # cv2.imshow("Red Mask", mask)
        # cv2.imshow("H Mask", h_mask)
        # cv2.waitKey(1)

        return h_mask, (cx, cy), largest_area