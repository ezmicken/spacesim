spacesim-osx:
	env GOOS=darwin go build -x -v -o ./bin/spacesim.dylib -buildmode=c-shared ./pkg/spacesim
