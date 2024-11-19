# ourFile.py

import karel


def main():
    pupper = karel.KarelPupper()

    #pupper.move() #it is ignoring this for some reason

    #its always 1 step forward

    pupper.move()
    pupper.turn_right()
    pupper.turn_right()
    #and 3 steps back
    pupper.move()
    pupper.move()
    pupper.move()
    pupper.turn_right()
    pupper.turn_right()
    #im the love of your life
    #until i make you mad
    

    

if __name__ == '__main__':
    main()
