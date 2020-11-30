spacesim-osx:
	env GOOS=darwin go build -o ./bin/spacesim.dylib -buildmode=c-shared ./pkg/spacesim
