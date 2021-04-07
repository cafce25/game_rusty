use game_rusty::GameRusty;

fn main() {
    let mut m = GameRusty::<10_000>::new();
    print!("{:?}", m.next_instruction())
}
