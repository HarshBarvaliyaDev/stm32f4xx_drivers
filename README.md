to run this drivers on the stm32cubeide . 
first create a project 

files > new > stm32 project

![image](https://github.com/HarshBarvaliyaDev/stm32f4xx_drivers/assets/107980330/8ddb6840-48d5-4148-bab3-4793dba3d095)

this drivers are for stm32f407xx series microcontrollers , so choose any microcontroller that starts with "stm32f407". i am choosing stm32f407vet6

![image](https://github.com/HarshBarvaliyaDev/stm32f4xx_drivers/assets/107980330/cd78e42a-ce56-47aa-9384-8bb6587c6c9d)

click next. 

and it will pop up an dialogue box.

![image](https://github.com/HarshBarvaliyaDev/stm32f4xx_drivers/assets/107980330/590b41f5-8f34-45f6-8f3a-9be51a8aba78)
as this is bare metal code . which is not dependent on any library . i recommend to choose "empty" in "targeted project type" field
give any name you want for the project , i named it "stm32_drivers".

![image](https://github.com/HarshBarvaliyaDev/stm32f4xx_drivers/assets/107980330/51928db2-48e2-4031-8bdb-bead4a06e0f7)

--- follow next steps for adding this drivers in your projects.

clone the git repository.
and paste both the directories ( drivers , Src ) in project directory.

right click on the project and go to properties.
![image](https://github.com/HarshBarvaliyaDev/stm32f4xx_drivers/assets/107980330/310825a8-9fd7-4d08-add2-69027979856a)


go to  " C/C++ build > settings > MCU GCC compiler > include paths "

![image](https://github.com/HarshBarvaliyaDev/stm32f4xx_drivers/assets/107980330/c5c913ff-2271-49a2-98ef-cbd36759b5e2)

in include path there should be only one path , which would be "../Inc"

click on "add" and then on "workspace"

![image](https://github.com/HarshBarvaliyaDev/stm32f4xx_drivers/assets/107980330/b3f18cf1-d466-482e-8670-d6ed780a33f6)

select "stm32_driver" and then add "driver/inc" and "driver/src" one by one

![image](https://github.com/HarshBarvaliyaDev/stm32f4xx_drivers/assets/107980330/d34742f1-85b1-435b-8df5-65528e960e49)

final "include paths" should look like this.

![image](https://github.com/HarshBarvaliyaDev/stm32f4xx_drivers/assets/107980330/28f89433-c5db-4b36-aa79-f1a2c967ca40)

click "apply" and then close.












