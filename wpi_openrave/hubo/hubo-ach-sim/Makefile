default: hubo-ach-openhubo

hubo-ach-openhubo: hubo-ach-openhubo.cpp
	g++ -I /usr/include/openrave-0.8/ -I ../../../wpi_hubo/hubo-ach/include -o $@ $< -lach -lrt -lboost_thread -lopenrave0.8 -lopenrave0.8-core
clean: 
	rm hubo-ach-openhubo
