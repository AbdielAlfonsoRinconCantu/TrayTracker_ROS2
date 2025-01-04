from asyncua import Client, Node, ua
import asyncio
import random

# PLC_url = "opc.tcp://HP-Omen-15-2016-Abdiel:4840"
PLC_url = "opc.tcp://192.168.0.114:4840"
print("\n Connecting to PLC...")

async def asyncio_main():
    async with Client(url=PLC_url) as client:
    
        random_list = [random.randint(0, 1) for _ in range(169)]
        print(random_list)
        
        for i in range(169):
            PLC_variable_path = f"0:Objects/2:DeviceSet/4:CODESYS Control Win V3 x64/3:Resources/4:Application/3:Programs/4:PLC_PRG/4:section{i+1}"
            PLC_variable = await client.nodes.root.get_child(f"{PLC_variable_path}")
            
            if random_list[i] == 0:
                await PLC_variable.write_value(False)
            else :
                await PLC_variable.write_value(True)
        
if __name__ == "__main__":
    asyncio.run(asyncio_main())
