
default:
	g++ -std=c++17 -Wall Main.cpp -lsfml-window -lsfml-graphics -lsfml-system -o kvadrati

run: default
	./kvadrati

clean:
	rm kvadrati
