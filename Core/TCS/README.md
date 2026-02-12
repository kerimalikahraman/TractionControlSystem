# STM32H7 Traction Control System (TCS) Architecture

Bu modál, Formula Student aracı için tasarlanmış deterministik, modüler ve güvenli bir TCS altyapısıdır.

## 📂 Dosya Yapısı (`Core/TCS`)

- **Inc/tcs_types.h**: Tüm veri tipleri, structlar, enumlar ve konfigürasyon parametreleri.
- **Src/tcs_input_cond.c**: Sensör verilerinin okunması, filtrelenmesi (LPF) ve zaman aşımı kontrolü.
- **Src/tcs_mode_mgr.c**: Sürüş moduna (AC, Skidpad, vb.) göre hedef slip (λ) belirlenmesi.
- **Src/tcs_vx_estimator.c**: Araç hızı tahmini (`min(wFL, wFR)`).
- **Src/tcs_slip_calc.c**: Arka teker slip oranlarının hesaplanması.
- **Src/tcs_controller_core.c**: P-kontrolcü ve tork limitleme mantığı. Hard-cut buradadır.
- **Src/tcs_safety_supervisor.c**: Sistem güvenliği, timeout ve limit aşımlarının takibi. Fault yönetimi.
- **Src/tcs_logger.c**: Debug ve NN veri toplama için CAN paketleme.
- **Src/tcs_app_main.c**: Ana görev zamanlayıcısı (Task Scheduler).

## 🚀 Entegrasyon Kılavuzu

`Core/Src/main.c` dosyanıza aşağıdaki adımları uygulayın:

1.  **Include Ekleyin:**
    ```c
    /* Private includes */
    #include "tcs_app_main.h"
    #include "tcs_input_cond.h" // Sensör güncelleme fonksiyonları için
    ```

2.  **Init Çağırın:**
    ```c
    /* USER CODE BEGIN 2 */
    TCS_Init();
    /* USER CODE END 2 */
    ```

3.  **Döngüye Ekleyin (10ms Task):**
    ```c
    /* USER CODE BEGIN WHILE */
    uint32_t last_tick = 0;
    while (1)
    {
        if (HAL_GetTick() - last_tick >= 10) // 100 Hz
        {
            last_tick = HAL_GetTick();
            TCS_Step(); 
            
            // CAN Gönderimi (Opsiyonel buraya veya Logger içine)
            // Logger_GetFrame(...)
        }
    /* USER CODE END WHILE */
    ```

4.  **Verileri Besleyin (HAL Callbackler veya Polling içinde):**
    ```c
    // Örnek: CAN Rx Callback içinde
    InputCond_UpdateWheelSpeeds(TCS_GetHandle(), fl, fr, rl, rr);
    InputCond_UpdateIMU(TCS_GetHandle(), ax, ay, yaw);
    InputCond_UpdateTorqueRequest(TCS_GetHandle(), t_req, t_max);
    ```

## 🧠 Neural Network (NN) Hazırlığı

Bu mimari NN entegrasyonuna şu şekilde hazırdır:

1.  **Veri Toplama:** `tcs_stat_t` yapısı içinde `slip_derivative` gibi featurelar hesaplanmaktadır. `tcs_logger.c` gerekli etiketleri (Target Torque, Actual Slip, vb.) CAN üzerinden basar.
2.  **Entegrasyon:** İleride `tcs_controller_core.c` içine bir NN inference motoru (ör. TFLite Micro) eklendiğinde:
    - Input: `state.slip_ratio`, `state.slip_derivative`, `inputs.accel_x`
    - Output: `state.target_slip` (Dynamic Target) veya doğrudan `limited_torque_nm`.
    - Deterministik yapı bozulmadan `ControllerCore_Update` içinde NN çağrılabilir.

## ⚠️ Güvenlik

- **Timeout:** Sensör verileri 100ms (ayarlanabilir) içinde güncellenmezse TCS devre dışı kalır.
- **Hard Cut:** Slip oranı > 0.25 (ayarlanabilir) olursa tork anında kesilir.
- **Fault Latch:** Hata durumunda sistem "Active" durumdan çıkar, reset gerektirebilir.

