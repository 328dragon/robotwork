#include "bsp_fdcan.h"

/* can instance ptrs storage, used for recv callback */
// 在CAN产生接收中断会遍历数组,选出hcan和rxid与发生中断的实例相同的那个,调用其回调函数
// @todo: 后续为每个CAN总线单独添加一个can_instance指针数组,提高回调查找的性能
static FDCANInstance *fdcan_instance[FDCAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief 添加过滤器以实现对特定id的报文的接收,会被CANRegister()调用
 *        给CAN添加过滤器后,BxCAN会根据接收到的报文的id进行消息过滤,符合规则的id会被填入FIFO触发中断
 *
 * @note f407的bxCAN有28个过滤器,这里将其配置为前14个过滤器给CAN1使用,后14个被CAN2使用
 *       初始化时,奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
 *       注册到CAN1的模块使用过滤器0-19,CAN2使用过滤器20-39
 *
 * @attention 你不需要完全理解这个函数的作用,因为它主要是用于初始化,在开发过程中不需要关心底层的实现
 *            享受开发的乐趣吧!如果你真的想知道这个函数在干什么,请联系作者或自己查阅资料(请直接查阅官方的reference manual)
 *
 * @param _instance can instance owned by specific module
 */
static void FDCANAddFilter(FDCANInstance *_instance)
{
    FDCAN_FilterTypeDef fdcan_filter_conf;
    //滤波器有0-63，共64个，准备将0-19给can1,20-39给can2,40-59给can3
    static uint8_t fdcan1_filter_idx = 0, fdcan2_filter_idx = 20,fdcan3_filter_idx=40; // 0-13给can1用,14-27给can2用

 // 配置RX滤波器
    fdcan_filter_conf.IdType = FDCAN_STANDARD_ID;  // 标准ID                        
    fdcan_filter_conf.FilterConfig =  (_instance->tx_id & 1) ?FDCAN_FILTER_TO_RXFIFO0:FDCAN_FILTER_TO_RXFIFO1;           // 过滤器0关联到FIFO0
    fdcan_filter_conf.FilterIndex =  _instance->fdcan_handle == &hfdcan1 ?(fdcan1_filter_idx++) : (fdcan2_filter_idx++);                                  // 滤波器索引
    fdcan_filter_conf.FilterType =FDCAN_FILTER_RANGE;                  // 滤波器类型
    fdcan_filter_conf.FilterID1 = 0x000;                      //    如果FDCAN配置为传统模式的话，这里是32位
    fdcan_filter_conf.FilterID2 = 0x7fff;                         //  全收     

    HAL_FDCAN_ConfigFilter(_instance->fdcan_handle, &fdcan_filter_conf);
}

/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动CAN1和CAN2,开启CAN1和CAN2的FIFO0 & FIFO1溢出通知
 *s
 */
static void FDCANServiceInit()
{
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);
}

/* ----------------------- two extern callable function -----------------------*/

FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config)
{
    if (!idx)
    {
        FDCANServiceInit(); // 第一次注册,先进行硬件初始化
        // LOGINFO("[bsp_can] CAN Service Init");
    }
    if (idx >= FDCAN_MX_REGISTER_CNT) // 超过最大实例数
    {
        while (1)
				{
                    Error_Handler();
                    // LOGERROR("[bsp_fdcan] FDCAN instance exceeded MAX num, consider balance the load of CAN bus");
					}
            
    }

    for (size_t i = 0; i < idx; i++)
    { // 重复注册 | id重复
        if (fdcan_instance[i]->rx_id == config->rx_id && fdcan_instance[i]->fdcan_handle == config->fdcan_handle)
        {
            while (1)
						{ 
                            
                               // LOGERROR("[}bsp_fdcan] FDCAN id crash ,tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
						}
                
        }
    }

    FDCANInstance *instance = (FDCANInstance *)malloc(sizeof(FDCANInstance)); // 分配空间
    memset(instance, 0, sizeof(FDCANInstance));                           // 分配的空间未必是0,所以要先清空
		
    // 进行发送报文的配置
    instance->txconf.Identifier = config->tx_id; // 发送id
    instance->txconf.IdType =FDCAN_STANDARD_ID;      // 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
		instance->txconf.FDFormat=FDCAN_CLASSIC_CAN;
		instance->txconf.TxFrameType= FDCAN_DATA_FRAME;   // 数据帧
    instance->txconf.DataLength = 0x08;            // 默认发送长度为8
    instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    instance->txconf.BitRateSwitch = FDCAN_BRS_OFF;           // 关闭速率切换
    instance->txconf.FDFormat = FDCAN_CLASSIC_CAN;            // 传统的CAN模式
    instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 无发送事件
    instance->txconf.MessageMarker = 0;

    // 设置回调函数和接收发送id
    instance->fdcan_handle = config->fdcan_handle;
    instance->tx_id = config->tx_id; // 好像没用,可以删掉
    instance->rx_id = config->rx_id;
    instance->fdcan_module_callback = config->fdcan_module_callback;
    instance->id = config->id;	   //模块id,没啥用
    FDCANAddFilter(instance);         // 添加CAN过滤器规则
    fdcan_instance[idx++] = instance; // 将实例保存到can_instance中

    return instance; // 返回can实例指针
}

/* @todo 目前似乎封装过度,应该添加一个指向tx_buff的指针,tx_buff不应该由CAN instance保存 */
/* 如果让CANinstance保存txbuff,会增加一次复制的开销 */
uint8_t FDCANTransmit(FDCANInstance *_instance)
{
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused)); // for cancel warning
//    float dwt_start = DWT_GetTimeline_ms();
    while ( HAL_FDCAN_GetTxFifoFreeLevel(_instance->fdcan_handle)== 0) // 等待邮箱空闲
    {      
//        if (DWT_GetTimeline_ms() - dwt_start > timeout) // 超时
//        {
//            //LOGWARNING("[bsp_can] CAN MAILbox full! failed to add msg to mailbox. Cnt [%lu]. Current instance hcan[%d] tx_id:[0x%x]", busy_count, _instance->can_handle == &hcan1 ? 1 : 2, _instance->txconf.StdId);
//            busy_count++;
//            // HAL_CAN_AbortTxRequest(_instance->can_handle,_instance->tx_mailbox);
//            return 0;
//        }
    }
//    wait_time = DWT_GetTimeline_ms() - dwt_start;
    // tx_mailbox会保存实际填入了这一帧消息的邮箱,但是知道是哪个邮箱发的似乎也没啥用
    if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->fdcan_handle, &_instance->txconf, _instance->tx_buff))
    {
//        LOGWARNING("[bsp_can] CAN bus BUS! cnt:%lu Current instance tx_id:[0x%x]", busy_count, _instance->txconf.StdId);
        busy_count++;
        // HAL_CAN_AbortTxRequest(_instance->can_handle,_instance->tx_mailbox);
        return 0;
    }
    return 1; // 发送成功
}

void FDCANSetDLC(FDCANInstance *_instance, uint8_t length)
{
    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0) // 安全检查
		{
				while (1) {
		//				 LOGERROR("[bsp_fdcan] CAN DLC error! check your code or wild pointer");
				}
	}
    _instance->txconf.DataLength = length;
}

/* -----------------------belows are callback definitions--------------------------*/

/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _hcan
 * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
 */
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox)
{
    static FDCAN_RxHeaderTypeDef rxconf; // 同上
    uint8_t fdcan_rx_buff[8];
    while (	HAL_FDCAN_GetRxFifoFillLevel(_hfdcan,fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
		
			HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, fdcan_rx_buff)  ;  // 从FIFO中获取数据
        for (size_t i = 0; i < idx; ++i)
        { // 两者相等说明这是要找的实例
            if (_hfdcan == fdcan_instance[i]->fdcan_handle && rxconf.RxFrameType == fdcan_instance[i]->rx_id)
            {
                if (fdcan_instance[i]->fdcan_module_callback != NULL) // 回调函数不为空就调用
                {
                    fdcan_instance[i]->rx_len = rxconf.DataLength;                      // 保存接收到的数据长度
                    memcpy(fdcan_instance[i]->rx_buff, fdcan_rx_buff, rxconf.DataLength); // 消息拷贝到对应实例
                    fdcan_instance[i]->fdcan_module_callback(fdcan_instance[i]);     // 触发回调进行数据解析和处理
                }
                return;
            }
						
        }
    }
}

/**
 * @brief 注意,STM32的两个CAN设备共享两个FIFO
 * 下面两个函数是HAL库中的回调函数,他们被HAL声明为__weak,这里对他们进行重载(重写)
 * 当FIFO0或FIFO1溢出时会调用这两个函数
 */
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
    
}
