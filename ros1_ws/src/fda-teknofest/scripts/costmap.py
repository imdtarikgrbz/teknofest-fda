import numpy as np
import cv2


class CostMap:

    def __init__(self, fx:float, fy:float, cx:float, cy:float):     
        """ Her frame de calculate_h'dan gelen H matrisi update fonksiyonuna verilerek çalıştırılır. Nesneyi her frame de yeniden oluşturmayın !!!
        update ve calculate_h her frame de çağırın.

        Args:
            fx (float): Kameranın x eksenindeki odak uzaklığı.
            fy (float): Kameranın y eksenindeki odak uzaklığı.
            cx (float): Kamera görüntüsündeki asal noktanın x koordinatı.
            cy (float): Kamera görüntüsündeki asal noktanın y koordinatı.
        """
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        
        # Kamera intrinsic matrisi
        self.K = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float64)

        # Costmap
        self.SIZE = 1200 # Costmapin bir kenarındaki grid sayısı
        self.RESOLUTION = 0.1 # metre / piksel
        self.CENTER = np.array([self.SIZE/2, self.SIZE/2])
        self.DEFAULT_COST = 128 # Varsayılan Maliyet
        self.MIN_ALTITUDE = 10 # H matrisinin hesaplanması için gerekli minimum irtifa (metre)

        # İKA'nın dünya/yerel koordinatı, Costmapin merkez hücresinin konumu, yada rotanın oluşturulmaya başlanacağı referans nokta
        self.ika_x = 0.0
        self.ika_y = 0.0

        # Camera -> Body !!! Kameranın konumuna göre daha sonra kontrol edilmeli !!!
        self.R_bc = np.array([
            [0,  -1,  0],
            [-1, 0,  0],
            [0,  0, -1]
        ], dtype=np.float64)
        
        self.patch = np.zeros(shape=(self.SIZE,self.SIZE), dtype=np.uint8)
        self.guven_haritasi = np.zeros(shape=(self.SIZE,self.SIZE), dtype=np.int8)
        self.costmap = np.full(shape=(self.SIZE,self.SIZE), fill_value=self.DEFAULT_COST, dtype=np.uint8) # varsayılan maliyet 128
        
    def update(self, mask:np.ndarray, H:np.ndarray):
        """YOLO maskesini homografi kullanarak costmap koordinat sistemine aktarır ve elde edilen sınıflandırma sonucuna göre güven haritasını ve maliyet haritasını günceller.

        Args:
            mask (np.ndarray): YOLO segmentasyon sonucunda elde edilen sınıf maskesi.
            H (np.ndarray): ``calculate_H`` fonksiyonundan elde edilen 3x3 homografi matrisi.

        Returns:
            _type_: CostMap
        """        
        if H is None:
            print("H matrisi None döndü, update başarısız")
            return None

        self.patch = cv2.warpPerspective(
            mask,
            H,
            (self.SIZE, self.SIZE),
            flags=cv2.INTER_NEAREST
        )
        
        # Güven Haritası, 1:road, 2:not_road, 3:target
        self.guven_haritasi[(self.patch == 1) | (self.patch == 3)] -= 1 # road için 1 azalt
        self.guven_haritasi[self.patch == 2] += 2 # not_road için 2 artır
        self.guven_haritasi = np.clip(self.guven_haritasi, -100, 100)
        
        # CostMapin oluşturulması
        self.costmap[self.guven_haritasi >= 4] = 255 # not_road maliyeti
        self.costmap[self.guven_haritasi <= -2] = 0 # road maliyeti
        self.costmap[(self.guven_haritasi < 4) & (self.guven_haritasi > -2)] = 128 # emin olunmayan karelerin maliyeti
        
        
        return self.costmap

    def calculate_H(
        self,
        roll:float,
        pitch:float,
        yaw:float,
        uav_x:float,
        uav_y:float,
        altitude:float
    ) -> np.ndarray:
        """Kamera görüntüsünü İKA merkezli costmap koordinat sistemine dönüştüren homografi matrisini hesaplar.

        Args:
            roll (float): İHA'nın x ekseni etrafındaki dönüş açısı (radyan).
            pitch (float): İHA'nın y ekseni etrafındaki dönüş açısı (radyan).
            yaw (float): İHA'nın z ekseni etrafındaki dönüş açısı (radyan).
            uav_x (float): İHA'nın İKA'ya göre dünya / yerel koordinat sistemindeki x konumu (metre).
            uav_y (float): İHA'nın İKA'ya göre dünya / yerel koordinat sistemindeki y konumu (metre).
            altitude (float): Kameranın zemin düzlemine olan dikey uzaklığı (metre).

        Returns:
            np.ndarray: Görüntü koordinatlarını İKA merkezli costmap koordinatlarına dönüştüren 3x3 homografi matrisi.
        """        

        if altitude < self.MIN_ALTITUDE:
            print("İrtifa beklenen değerden az, matris hesaplanmadı !!!") #!
            return None
        
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        Rx = np.array([
            [1, 0, 0],
            [0, cr, -sr],
            [0, sr, cr]
        ])

        Ry = np.array([
            [cp, 0, sp],
            [0, 1, 0],
            [-sp, 0, cp]
        ])

        Rz = np.array([
            [cy, -sy, 0],
            [sy,  cy, 0],
            [0,    0, 1]
        ])

        # Body -> World
        R_wb = Rz @ Ry @ Rx

        # Camera -> World
        R_wc = R_wb @ self.R_bc

        # World -> Camera
        R_cw = R_wc.T

        # İHA'nın dünya koordinatı
        C = np.array([
            uav_x,
            uav_y,
            altitude
        ])

        # World -> Camera translation
        t_cw = -R_cw @ C

        # Z = 0 zemin düzlemi için:
        # World -> Image
        H_world_to_image = self.K @ np.column_stack((
            R_cw[:, 0],
            R_cw[:, 1],
            t_cw
        ))

        # Image -> World ground
        H_image_to_ground = np.linalg.inv(
            H_world_to_image
        )

        # World ground -> İKA merkezli costmap
        M_ground_to_map = np.array([
            [
                1 / self.RESOLUTION,
                0,
                self.CENTER[0] - self.ika_x / self.RESOLUTION
            ],
            [
                0,
                -1 / self.RESOLUTION,
                self.CENTER[1] + self.ika_y / self.RESOLUTION
            ],
            [
                0, 0, 1
            ]
        ])

        # Image -> Costmap
        H = M_ground_to_map @ H_image_to_ground

        # Normalize
        H /= H[2, 2]

        return H
