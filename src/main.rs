use game_rusty::engine::Engine;

fn main() {
    let mut m = Engine::<10_000, 10_000>::new();
    print!("{:?}", m.next_instruction())
}
