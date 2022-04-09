// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ToolShed;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



/**Command Array*/
public class CommandArray {
    private ArrayList<Command> commands = new ArrayList<Command>();
    private ArrayList<String> names = new ArrayList<String>();
    private String name;

    public CommandArray(String name)
    {
        this.name = name;
    }

    public CommandArray(String name, Command[] command)
    {
        this.name = name;
        addCommands(command);
    }

    public void addCommand(Command command)
    {
        commands.add(command);
        names.add(command.getName());
    }

    public void addCommand(Command command, String name)
    {
        commands.add(command);
        names.add(name);
    }

    public void addCommands(Command... command)
    {
        for(Command e : command)
        {
            commands.add(e);
            names.add(e.getName());
        }
    }

    public void addCommands(String[] nameCom, Command... command)
    {
        for(int i = 0; i < nameCom.length; i++)
        {
            commands.add(command[i]);
            names.add(nameCom[i]);
        }
    }

     /**
     * "Creates a new ParallelCommandGroup. The given commands will be executed simultaneously. The command group will finish when the last command finishes. If the CommandGroup is interrupted, only the commands that are still running will be interrupted."
     * @param command commands
     */
    public void addParallelCommandGroup(Command... command)
    {
        commands.add(new ParallelCommandGroup(command));
        for(Command e : command)
        {
            names.add(e.getName());
        }
    }

    /**
     * "Creates a new ParallelCommandGroup. The given commands will be executed simultaneously. The command group will finish when the last command finishes. If the CommandGroup is interrupted, only the commands that are still running will be interrupted."
     * @param namePar array of string names
     * @param command commands
     */
    public void addParallelCommandGroup(String[] namePar, Command... command)
    {
        commands.add(new ParallelCommandGroup(command));
        for(String e : namePar)
        {
            names.add(e);
        }
    }






    /**
     * "Creates a new ParallelDeadlineGroup. The given commands (including the deadline) will be executed simultaneously. The CommandGroup will finish when the deadline finishes, interrupting all other still-running commands. If the CommandGroup is interrupted, only the commands still running will be interrupted."
     * @param deadline the command that determines when the group ends
     * @param commands the commands to be executed
     */
    public void addParallelDeadlineCommandGroup(Command deadline, Command... command)
    {
        commands.add(new ParallelDeadlineGroup(deadline, command));
        for(Command e : command)
        {
            names.add(e.getName());
        }
    }

    /**
     * "Creates a new ParallelDeadlineGroup. The given commands (including the deadline) will be executed simultaneously. The CommandGroup will finish when the deadline finishes, interrupting all other still-running commands. If the CommandGroup is interrupted, only the commands still running will be interrupted."
     * @param namePar array of string names
     * @param deadline the command that determines when the group ends
     * @param commands the commands to be executed
     */
    public void addParallelDeadlineCommandGroup(String[] namePar, Command deadline, Command... command)
    {
        commands.add(new ParallelDeadlineGroup(deadline, command));
        for(String e : namePar)
        {
            names.add(e);
        }
    }






    public void setName(String name, String newName)
    {
        names.set(names.indexOf(name), newName);
    }

    public void setName(Command command, String newName)
    {
        names.set(commands.indexOf(command), newName);
    }

    public void setName(String name)
    {
        this.name = name;
    }

    public void removeCommand(String name)
    {
        commands.remove(names.indexOf(name));
        names.remove(names.indexOf(name));
    }

    public void removeCommand(Command command)
    {
        names.remove(commands.indexOf(command));
        commands.remove(commands.indexOf(command));
    }

    public Command getCommand(String name)
    {
        return commands.get(names.indexOf(name));
    }

    public ArrayList<Command> getCommandArrayList()
    {
        return commands;
    }

    public class addCommandss extends SequentialCommandGroup
    {
        public addCommandss()
        {
            addCommands(toCommandArray());
        }
    }

    /**
     * "Creates a new SequentialCommandGroup. The given commands will be run sequentially, with the CommandGroup finishing when the last command finishes."
     * @return new SequentialCommandGroup
     */
    public SequentialCommandGroup asSequentialCommandGroup()
    {
        return new SequentialCommandGroup(toCommandArray());
    }

    public Command[] toCommandArray()
    {
        return commands.toArray(new Command[commands.size()]);
    }

    public String getName(Command command)
    {
        return names.get(commands.indexOf(command));
    }

    public String getName()
    {
        return this.name;
    }

 /**idk if this works*/
    public String toString()
    {
        if(commands.isEmpty() || names.isEmpty())
        {
            return "crickets... crickets everywhere";
        }
        else 
        {
            String speech = this.name;
            speech += (" /n" + "/n");
            for(int i = 0; i < commands.size(); i++)
            {
                speech += ("Command " + (i+1) + ": " + names.get(i));
                speech += ("/n");
            }
            return speech;
        }
    }
}
