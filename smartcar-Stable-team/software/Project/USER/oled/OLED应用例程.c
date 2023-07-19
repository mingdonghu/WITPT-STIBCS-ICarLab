 uint16 Speed_Set=0;
   
   
 void main(void)
 { 
   OLED_Init();         //OLED显示初始化化
   OLED_Fill(0,0,127,63,0);//填充
   //显示队名
   OLED_ShowString(0,0,"HJM SmartCarTeam" ,16);//显示字符串    X:列  Y：行
   OLED_ShowString(0,25,"SetSpeedValue:" ,12);//显示字符串
   
   Speed_Set = 530;     //初始化设置车子速度
   OLED_ShowNum(100,25,(uint8)(Speed_Set/10),2,12);
   OLED_Refresh_Gram();//更新显存到OLED 
   
   while;
   return;
 }